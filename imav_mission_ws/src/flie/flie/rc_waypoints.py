#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from pynput import keyboard
import time
import os

class TeleopSaverNode(Node):
    def __init__(self):
        super().__init__('teleop_saver_node')
        
        # Publicador de velocidad (Reactivo)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Suscriptor a la posición del dron (para guardar la trayectoria)
        # Asegúrate de cambiar 'pose_topic' por el tópico real donde publicas mi posición
        self.pose_sub = self.create_subscription(PoseStamped, '/cf_pose', self.pose_callback, 10)
        
        # Archivo para guardar la trayectoria
        self.archivo_trayectoria = open("trayectoria_guardada.txt", "w", encoding="utf-8")
        self.get_logger().info("¡Grabación de trayectoria iniciada!")

        # Variables de velocidad actual
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.vel_yaw = 0.0
        
        # Parámetros de velocidad (Pequeños para que sea muy reactivo y preciso)
        self.speed = 0.2
        self.turn = 0.5
        
        # Timer para publicar constantemente (Watchdog friendly a 10Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Variable para la posición actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Iniciar el Listener del teclado
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def pose_callback(self, msg):
        # Actualizamos mi posición actual en memoria
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.vel_x = self.speed
            elif key.char == 's':
                self.vel_x = -self.speed
            elif key.char == 'a':
                self.vel_y = self.speed
            elif key.char == 'd':
                self.vel_y = -self.speed
        except AttributeError:
            # Teclas especiales (Flechas)
            if key == keyboard.Key.up:
                self.vel_z = self.speed
            elif key == keyboard.Key.down:
                self.vel_z = -self.speed
            elif key == keyboard.Key.left:
                self.vel_yaw = self.turn
            elif key == keyboard.Key.right:
                self.vel_yaw = -self.turn
            elif key == keyboard.Key.space:
                # Botón para GUARDAR EL PUNTO ACTUAL
                self.guardar_punto()

    def on_release(self, key):
        # Cuando sueltas la tecla, esa velocidad en específico se vuelve CERO (Hold)
        try:
            if key.char in ['w', 's']:
                self.vel_x = 0.0
            elif key.char in ['a', 'd']:
                self.vel_y = 0.0
        except AttributeError:
            if key in [keyboard.Key.up, keyboard.Key.down]:
                self.vel_z = 0.0
            elif key in [keyboard.Key.left, keyboard.Key.right]:
                self.vel_yaw = 0.0
            elif key == keyboard.Key.esc:
                # Salir del programa limpiamente
                self.archivo_trayectoria.close()
                self.get_logger().info("Trayectoria cerrada y guardada.")
                return False

    def publish_velocity(self):
        # Siempre publicamos, si no hay teclas presionadas publicará 0,0,0 (Hold)
        twist = Twist()
        twist.linear.x = self.vel_x
        twist.linear.y = self.vel_y
        twist.linear.z = self.vel_z
        twist.angular.z = self.vel_yaw
        self.cmd_vel_pub.publish(twist)

    def guardar_punto(self):
        # Guarda la coordenada actual en el archivo
        linea = f"{self.current_x:.3f},{self.current_y:.3f},{self.current_z:.3f}\n"
        self.archivo_trayectoria.write(linea)
        self.archivo_trayectoria.flush()
        self.get_logger().info(f"📍 Punto Guardado: {linea.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.archivo_trayectoria.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
