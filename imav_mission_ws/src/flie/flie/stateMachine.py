#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point

# Definición de los Estados de la Misión
STATE_INIT = 0
STATE_TAKEOFF = 1
STATE_NAVIGATING = 2
STATE_LANDING = 3
STATE_DONE = 4

class CFBrainNode(Node):
    def __init__(self):
        super().__init__('cf_brain_node')
        
        # Publicadores
        self.takeoff_pub = self.create_publisher(Bool, 'hardware/start_takeoff', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.landing_pub = self.create_publisher(Bool, 'hardware/start_landing', 10)
        self.target_pub = self.create_publisher(Point, 'navigator/set_target', 10)
        self.nav_enable_pub = self.create_publisher(Bool, 'navigator/enable', 10)

        # Suscriptores
        self.ready_sub = self.create_subscription(Bool, 'hardware/takeoff_ready', self.ready_cb, 10)
        self.arrived_sub = self.create_subscription(Bool, 'navigator/arrived', self.arrived_cb, 10)
        
        # Variables de estado
        self.is_ready = False
        self.has_arrived = False
        
        # Lógica de la Máquina de Estados
        self.state = STATE_INIT
        self.command_sent = False  # Evita que publiquemos la misma orden 10 veces por segundo
        
        self.waypoints = [
            (0.0, -1.0, 1.2),
            (2.0, -1.0, 1.2),
            (1.0, -0.5, 1.0),
            (2.0, -0.5, 1.0)
        ]
        self.current_waypoint_index = 0

        # Timer principal a 10 Hz (ejecuta control_loop cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Brain Node Iniciado. Esperando para arrancar misión...")

    def ready_cb(self, msg: Bool):
        if msg.data:
            self.is_ready = True
            self.get_logger().info("¡Dron reporta que está listo/en el aire!")

    def arrived_cb(self, msg: Bool):
        if msg.data:
            self.has_arrived = True

    def control_loop(self):
        """Esta es tu Máquina de Estados. Se ejecuta automáticamente cada 0.1s"""
        
        # ESTADO 0: Inicialización
        if self.state == STATE_INIT:
            self.get_logger().info("Iniciando secuencia en 2 segundos...")
            # Simulamos un pequeño retraso inicial usando el reloj en lugar de sleep
            self.state = STATE_TAKEOFF

        # ESTADO 1: Despegue
        elif self.state == STATE_TAKEOFF:
            if not self.command_sent:
                self.get_logger().info("Enviando orden de despegue...")
                self.takeoff_pub.publish(Bool(data=True))
                self.command_sent = True
            
            # Condición para avanzar de estado
            if self.is_ready:
                self.state = STATE_NAVIGATING
                self.command_sent = False # Reseteamos la bandera para el siguiente estado

        # ESTADO 2: Navegación por puntos
        elif self.state == STATE_NAVIGATING:
            if not self.command_sent:
                # Obtenemos la coordenada actual de la lista
                x, y, z = self.waypoints[self.current_waypoint_index]
                
                # Activamos el navegador y enviamos el punto
                self.nav_enable_pub.publish(Bool(data=True))
                pto = Point()
                pto.x, pto.y, pto.z = float(x), float(y), float(z)
                
                self.get_logger().info(f"Volando hacia objetivo {self.current_waypoint_index + 1}: X:{x}, Y:{y}, Z:{z}")
                self.target_pub.publish(pto)
                
                self.has_arrived = False
                self.command_sent = True
            
            # Condición para avanzar al siguiente punto o aterrizar
            if self.has_arrived:
                self.get_logger().info(f"¡Objetivo {self.current_waypoint_index + 1} alcanzado!")
                self.current_waypoint_index += 1
                self.command_sent = False # Reseteamos para enviar el siguiente punto
                
                # Revisamos si ya terminamos la lista de coordenadas
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info("Todos los waypoints completados.")
                    self.state = STATE_LANDING

        # ESTADO 3: Aterrizaje
        elif self.state == STATE_LANDING:
            if not self.command_sent:
                self.get_logger().info("Enviando orden de aterrizaje...")
                self.landing_pub.publish(Bool(data=True))
                self.command_sent = True
                self.state = STATE_DONE

        # ESTADO 4: Misión Finalizada
        elif self.state == STATE_DONE:
            # Apagamos el timer para no gastar recursos una vez terminada la misión
            self.timer.cancel()
            self.get_logger().info("Misión finalizada con éxito.")

def main(args=None):
    rclpy.init(args=args)
    node = CFBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
