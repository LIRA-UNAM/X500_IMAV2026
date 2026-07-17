#!/usr/bin/env python3
"""
CrazyfliePosePublisher

Encapsula el reset del estimador Kalman y los LogConfig de
posición/orientación. Publica los datos crudos como un
geometry_msgs/PoseStamped en el tópico 'hardware/pose_raw' -- NO arma
ni publica tf directamente. Eso lo hace un nodo aparte
(tf_broadcaster_node.py), que solo se suscribe a este tópico.

No es un nodo ROS ni abre su propia conexión al Crazyflie -- recibe el
nodo (para logging/clock/publisher) y la conexión ya abierta (scf) por
fuera, porque el radio del Crazyflie solo permite una conexión activa
a la vez, y esa conexión ya la abre CFHardwareNode en prueba_avance.py.

Uso:
    pose_pub = CrazyfliePosePublisher(node)
    pose_pub.reset_estimator(scf)
    pose_pub.start_logging(scf)
"""
import time

from geometry_msgs.msg import PoseStamped
from cflib.crazyflie.log import LogConfig


class CrazyfliePosePublisher:
    def __init__(self, node, world_frame_id='world', topic='hardware/pose_raw'):
        self.node = node
        self.world_frame_id = world_frame_id

        self.pose_pub = node.create_publisher(PoseStamped, topic, 10)

        # Último valor conocido de cada bloque de log (se combinan al publicar)
        self._last_pos = (0.0, 0.0, 0.0)
        self._last_quat = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
        self._got_first_pos = False
        self._got_first_quat = False

        # Referencias a los LogConfig para que no las recolecte el GC
        self._log_pos = None
        self._log_quat = None

    def reset_estimator(self, scf):
        """
        Resetea el estimador Kalman y espera a que converja. Llamar
        justo después de conectar, para que el origen 'world' sea la
        posición física donde estaba el dron al arrancar el nodo.
        """
        self.node.get_logger().info("Reseteando estimador Kalman...")
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2.0)  # tiempo para que el filtro converja
        self.node.get_logger().info("Estimador reseteado. Origen fijado en la posición actual.")

    def start_logging(self, scf):
        """
        Arranca los LogConfig de posición y orientación. Llamar una vez
        que la conexión (scf) ya está abierta, sin esperar al takeoff.

        Nota: un solo bloque de log tiene un límite de ~26 bytes de
        payload por paquete CRTP. 3 floats (posición) + 4 floats
        (cuaternión) = 28 bytes excede ese límite, por eso se dividen
        en dos bloques de 12 y 16 bytes.
        """
        log_pos = LogConfig(name='KalmanPos', period_in_ms=100)  # 10 Hz
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        scf.cf.log.add_config(log_pos)
        log_pos.data_received_cb.add_callback(self._position_callback)
        log_pos.error_cb.add_callback(self._log_error_cb)
        log_pos.start()

        log_quat = LogConfig(name='KalmanQuat', period_in_ms=100)  # 10 Hz
        log_quat.add_variable('kalman.q0', 'float')
        log_quat.add_variable('kalman.q1', 'float')
        log_quat.add_variable('kalman.q2', 'float')
        log_quat.add_variable('kalman.q3', 'float')
        scf.cf.log.add_config(log_quat)
        log_quat.data_received_cb.add_callback(self._orientation_callback)
        log_quat.error_cb.add_callback(self._log_error_cb)
        log_quat.start()

        self._log_pos = log_pos
        self._log_quat = log_quat

        self.node.get_logger().info("Kalman position/orientation logging started (10 Hz, 2 blocks)")

    # -- internos --

    def _log_error_cb(self, logconf, msg):
        self.node.get_logger().error(f"Log config '{logconf.name}' error: {msg}")

    def _position_callback(self, timestamp, data, logconf):
        if not self._got_first_pos:
            self._got_first_pos = True
            self.node.get_logger().info(f"Primer dato de posición recibido: {data}")
        self._last_pos = (
            data['kalman.stateX'],
            data['kalman.stateY'],
            data['kalman.stateZ'],
        )
        self._publish_pose()

    def _orientation_callback(self, timestamp, data, logconf):
        if not self._got_first_quat:
            self._got_first_quat = True
            self.node.get_logger().info(f"Primer dato de orientación recibido: {data}")
        # kalman.q0 = w, q1 = x, q2 = y, q3 = z
        self._last_quat = (
            data['kalman.q0'],
            data['kalman.q1'],
            data['kalman.q2'],
            data['kalman.q3'],
        )
        self._publish_pose()

    def _publish_pose(self):
        """Publica un PoseStamped crudo -- NO es tf, es solo el dato."""
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame_id

        x, y, z = self._last_pos
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        w, qx, qy, qz = self._last_quat
        msg.pose.orientation.w = w
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz

        self.pose_pub.publish(msg)
