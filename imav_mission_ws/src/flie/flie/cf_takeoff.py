#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import logging

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from flie.pose_publisher import CrazyfliePosePublisher

URI = 'radio://0/80/2M'

# Estados del Dron
STATE_DISCONNECTED = 0
STATE_WAIT_TAKEOFF = 1
STATE_FLYING = 2
STATE_LANDING = 3

class CFHardwareNode(Node):
    def __init__(self):
        super().__init__('cf_hardware_node')
        
        # Parámetros
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('pose_topic', 'hardware/pose_raw')

        # Suscriptores
        self.create_subscription(Bool, 'hardware/start_takeoff', self.takeoff_cb, 10)
        self.create_subscription(Twist, 'hardware/cmd_vel_cf', self.vel_cb, 10)
        self.create_subscription(Bool, 'hardware/start_landing', self.land_cb, 10)

        # Publicadores
        self.takeoff_rdy_pub = self.create_publisher(Bool, 'hardware/takeoff_ready', 10)
        self.pose_publisher = CrazyfliePosePublisher(
            self, 
            world_frame_id=self.get_parameter('world_frame_id').value, 
            topic=self.get_parameter('pose_topic').value
        )

        # Variables de Control y Estado
        self.state = STATE_DISCONNECTED
        self.takeoff_requested = False
        self.land_requested = False
        
        # Objetos de conexión
        self.scf = None
        self.mc = None

        # Timer principal de ROS 2 (se ejecuta a 10 Hz / cada 0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Hardware node iniciado. Esperando primera iteración del timer...")

    def takeoff_cb(self, msg: Bool):
        if msg.data and not self.takeoff_requested:
            self.takeoff_requested = True
            self.get_logger().info("¡Orden de despegue recibida!")

    def vel_cb(self, msg: Twist):
        if self.mc is not None and self.state == STATE_FLYING:
            self.mc.start_linear_motion(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def land_cb(self, msg: Bool):
        if msg.data and not self.land_requested:
            self.land_requested = True
            self.get_logger().info("¡Orden de aterrizaje recibida!")

    def control_loop(self):
        """Bucle de control ejecutado periódicamente por rclpy.spin()"""
        
        # ESTADO 0: Conectar al Crazyflie
        if self.state == STATE_DISCONNECTED:
            self.get_logger().info("Conectando con el Crazyflie...")
            self.scf = SyncCrazyflie(URI)
            self.scf.open_link()
                
            # Inicializar estimador y logs de pose
            self.pose_publisher.reset_estimator(self.scf)
            self.pose_publisher.start_logging(self.scf)

            self.state = STATE_WAIT_TAKEOFF
            self.get_logger().info("Conectado con éxito. Esperando señal en /hardware/start_takeoff...")

        # ESTADO 1: Esperando orden de despegue
        elif self.state == STATE_WAIT_TAKEOFF:
            if self.takeoff_requested:
                self.get_logger().info("Iniciando despegue...")
                self.mc = MotionCommander(self.scf)
                self.mc.take_off()

                # Notificar a ROS que el despegue terminó
                rdy_msg = Bool()
                rdy_msg.data = True
                self.takeoff_rdy_pub.publish(rdy_msg)

                self.state = STATE_FLYING
                self.get_logger().info("En el aire. Listo para recibir comandos cmd_vel.")

        # ESTADO 2: Volando
        elif self.state == STATE_FLYING:
            if self.land_requested:
                self.state = STATE_LANDING

        # ESTADO 3: Aterrizando y cerrando conexión
        elif self.state == STATE_LANDING:
            self.get_logger().info("Aterrizando...")
            if self.mc:
                self.mc.land()
                self.mc = None
            
            if self.scf:
                self.scf.close_link()
                self.scf = None

            self.get_logger().info("Aterrizaje completado y enlace cerrado.")
            # Desactivar el timer para dejar de ejecutar la lógica
            self.timer.cancel()

    def destroy_node(self):
        # Asegurar limpieza al apagar el nodo
        if self.mc:
            self.mc.land()
        if self.scf:
            self.scf.close_link()
        super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    rclpy.init(args=args)
    node = CFHardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()