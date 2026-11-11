import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Pose2D  # <- Importamos Pose2D para enviar las deltas
from tf_transformations import quaternion_multiply, euler_from_quaternion

_Q_NED_TO_ENU = (0.70710678, 0.70710678, 0.0, 0.0)
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Threshold parameters
        self.declare_parameter('distance_threshold', 0.2)
        self.declare_parameter('yaw_threshold', 0.3)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Suscripción a la odometría de PX4
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_cb,
            qos_profile
        )

        # Publicador de las deltas locales
        self.delta_pub = self.create_publisher(Pose2D, '/drone_deltas', 10)

        # Variables para valores previos
        self.prev_x = None
        self.prev_y = None
        # self.prev_z = None  # (Z omitido intencionalmente para la lógica 2D)
        self.prev_yaw = None
        
        # Logs de inicialización
        umbral_dist = self.get_parameter('distance_threshold').value
        umbral_yaw = self.get_parameter('yaw_threshold').value
        self.get_logger().info(
            f"Delta node start.\n"
            f"Distance threshold: {umbral_dist} meters (2D)\n"
            f"Yaw threshold: {umbral_yaw} radians"
        )

    def odometry_cb(self, msg: VehicleOdometry):
        # Extract current data and convert to ENU
        curr_x = msg.position[1]
        curr_y = msg.position[0]
        # curr_z = -msg.position[2] # Ignoramos Z en el cálculo para el filtro 2D

        q_cur = (msg.q[1], msg.q[2], msg.q[3], msg.q[0]) 
        q_enu = quaternion_multiply(quaternion_multiply(_Q_NED_TO_ENU, q_cur), _Q_FRD_TO_FLU)
        _, _, curr_yaw = euler_from_quaternion(q_enu)

        # Initialize the reference point
        if self.prev_x is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_yaw = curr_yaw
            self.get_logger().info(f"Starter point: X:{curr_x:.2f}, Y:{curr_y:.2f}, Yaw:{curr_yaw:.2f}")
            return

        # Calculate GLOBAL deltas (respecto al marco ENU)
        delta_x_global = curr_x - self.prev_x
        delta_y_global = curr_y - self.prev_y
        
        # 1. Distancia en 2D (Ignorando delta_z)
        distance_2d = math.hypot(delta_x_global, delta_y_global)

        delta_yaw = curr_yaw - self.prev_yaw
        # Normalize to a range [-pi, pi]
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        abs_delta_yaw = abs(delta_yaw)

        # Read args
        dist_threshold = self.get_parameter('distance_threshold').value
        yaw_threshold = self.get_parameter('yaw_threshold').value

        # Delta conditions
        if distance_2d >= dist_threshold or abs_delta_yaw >= yaw_threshold:
            
            # 2. Transformación Homogénea (Matriz de rotación inversa)
            # Pasamos las deltas globales al marco local (body) del dron usando el yaw previo
            dx_local = delta_x_global * math.cos(self.prev_yaw) + delta_y_global * math.sin(self.prev_yaw)
            dy_local = -delta_x_global * math.sin(self.prev_yaw) + delta_y_global * math.cos(self.prev_yaw)
            
            # 3. Publicamos los datos
            delta_msg = Pose2D()
            delta_msg.x = dx_local      # Avance/Retroceso
            delta_msg.y = dy_local      # Movimiento lateral (Strafe)
            delta_msg.theta = delta_yaw # Rotación
            self.delta_pub.publish(delta_msg)

            # Logs 
            Trigger = "DISTANCE" if distance_2d >= dist_threshold else "YAW"
            if distance_2d >= dist_threshold and abs_delta_yaw >= yaw_threshold:
                Trigger = "Both"

            self.get_logger().info(
                f"\n[Update by: {Trigger} | Dist 2D: {distance_2d:.3f}m, YAW: {abs_delta_yaw:.3f}rad]\n"
                f"Global -> dX: {delta_x_global:.4f}m, dY: {delta_y_global:.4f}m\n"
                f"Local  -> dx: {dx_local:.4f}m, dy: {dy_local:.4f}m, dyaw: {delta_yaw:.4f}rad"
            )

            # Update the reference point
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_yaw = curr_yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
