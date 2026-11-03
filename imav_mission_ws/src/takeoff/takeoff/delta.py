import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf_transformations import quaternion_multiply, euler_from_quaternion

# Rotaciones fijas para pasar de PX4 (NED) a ROS (ENU)
_Q_NED_TO_ENU = (0.70710678, 0.70710678, 0.0, 0.0)
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)

class OdomDeltaThresholdNode(Node):
    def __init__(self):
        super().__init__('odom_delta_threshold_node')

        # Umbral
        self.declare_parameter('distance_threshold', 0.1)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_cb,
            qos_profile
        )

        # Variables para almacenar el estado del punto de referencia anterior
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_yaw = None

        umbral_inicial = self.get_parameter('distance_threshold').value
        self.get_logger().info(f"Nodo iniciado. Umbral de distancia configurado a: {umbral_inicial} metros.")

    def odometry_cb(self, msg: VehicleOdometry):
        # Ignorar si el estimador aún no inicializa los datos (NaN)
        if any(math.isnan(v) for v in msg.position) or any(math.isnan(v) for v in msg.q):
            return

        # 1. Extraer posición actual y convertir de NED (PX4) a ENU (ROS)
        curr_x = msg.position[1]
        curr_y = msg.position[0]
        curr_z = -msg.position[2]

        # 2. Extraer yaw actual
        q_cur = (msg.q[1], msg.q[2], msg.q[3], msg.q[0]) 
        q_enu = quaternion_multiply(quaternion_multiply(_Q_NED_TO_ENU, q_cur), _Q_FRD_TO_FLU)
        _, _, curr_yaw = euler_from_quaternion(q_enu)

        # 3. Si es el primer dato que recibimos, inicializamos el punto de referencia y salimos
        if self.prev_x is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw
            self.get_logger().info("Punto de referencia inicial capturado.")
            return

        # 4. Calcular los deltas temporales (solo para medir la distancia)
        delta_x = curr_x - self.prev_x
        delta_y = curr_y - self.prev_y
        delta_z = curr_z - self.prev_z

        # Calcular la magnitud del vector desplazamiento (distancia euclidiana 3D)
        distancia = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        # Leer el umbral del parámetro (permite cambiarlo dinámicamente si usaras rqt_reconfigure)
        threshold = self.get_parameter('distance_threshold').value

        # 5. Evaluar si se superó el umbral
        if distancia >= threshold:
            # Calcular delta_yaw y normalizarlo a [-pi, pi]
            delta_yaw = curr_yaw - self.prev_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

            # Imprimir los valores
            self.get_logger().info(
                f"\n[Movimiento Detectado: {distancia:.3f} m]\n"
                f"Deltas -> dx: {delta_x:.4f}m, dy: {delta_y:.4f}m, dz: {delta_z:.4f}m, dyaw: {delta_yaw:.4f}rad"
            )

            # 6. ACTUALIZAR el punto de referencia
            # El punto actual se convierte en el nuevo punto anterior (se reinicia el contador de distancia)
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdomDeltaThresholdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()