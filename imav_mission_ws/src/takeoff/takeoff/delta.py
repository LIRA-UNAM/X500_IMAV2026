import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tf_transformations import quaternion_multiply, euler_from_quaternion

_Q_NED_TO_ENU = (0.70710678, 0.70710678, 0.0, 0.0)
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)

class OdomDeltaThresholdNode(Node):
    def __init__(self):
        super().__init__('odom_delta_threshold_node')

        # Declarar parámetros: 10 cm de distancia y ~10 grados (0.1745 rad) por defecto
        self.declare_parameter('distance_threshold', 0.1)
        self.declare_parameter('yaw_threshold', 0.1745)

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

        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_yaw = None
        
        self.warned_nan = False

        umbral_dist = self.get_parameter('distance_threshold').value
        umbral_yaw = self.get_parameter('yaw_threshold').value
        self.get_logger().info(
            f"Nodo iniciado.\n"
            f"  - Umbral de distancia: {umbral_dist} metros\n"
            f"  - Umbral de Yaw: {umbral_yaw} radianes"
        )
        self.get_logger().info("Esperando datos válidos de odometría...")

    def odometry_cb(self, msg: VehicleOdometry):
        # 1. Verificar si el EKF2 está publicando valores nulos
        if any(math.isnan(v) for v in msg.position) or any(math.isnan(v) for v in msg.q):
            if not self.warned_nan:
                self.get_logger().warn("Recibiendo 'NaN'. El estimador EKF2 no ha inicializado la posición.")
                self.warned_nan = True
            return
        
        self.warned_nan = False

        # 2. Extraer datos actuales y convertir a ENU
        curr_x = msg.position[1]
        curr_y = msg.position[0]
        curr_z = -msg.position[2]

        q_cur = (msg.q[1], msg.q[2], msg.q[3], msg.q[0]) 
        q_enu = quaternion_multiply(quaternion_multiply(_Q_NED_TO_ENU, q_cur), _Q_FRD_TO_FLU)
        _, _, curr_yaw = euler_from_quaternion(q_enu)

        # 3. Inicializar el punto de referencia
        if self.prev_x is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw
            self.get_logger().info(f"Punto inicial capturado: X:{curr_x:.2f}, Y:{curr_y:.2f}, Z:{curr_z:.2f}, Yaw:{curr_yaw:.2f}. Esperando movimiento...")
            return

        # 4. Calcular deltas temporales (magnitudes absolutas)
        delta_x = curr_x - self.prev_x
        delta_y = curr_y - self.prev_y
        delta_z = curr_z - self.prev_z
        distancia = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        delta_yaw = curr_yaw - self.prev_yaw
        # Normalizar a rango [-pi, pi]
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        giro_absoluto = abs(delta_yaw)

        # 5. Leer los umbrales dinámicamente
        dist_threshold = self.get_parameter('distance_threshold').value
        yaw_threshold = self.get_parameter('yaw_threshold').value

        # 6. Evaluar si ALGUNO de los dos umbrales se ha superado
        if distancia >= dist_threshold or giro_absoluto >= yaw_threshold:
            
            # Identificar qué activó el disparador (solo para impresión)
            motivo = "DISTANCIA" if distancia >= dist_threshold else "GIRO"
            if distancia >= dist_threshold and giro_absoluto >= yaw_threshold:
                motivo = "AMBOS"

            self.get_logger().info(
                f"\n[Actualización por: {motivo} | Dist: {distancia:.3f}m, Giro: {giro_absoluto:.3f}rad]\n"
                f"Deltas -> dx: {delta_x:.4f}m, dy: {delta_y:.4f}m, dz: {delta_z:.4f}m, dyaw: {delta_yaw:.4f}rad"
            )

            # 7. Actualizar el punto de referencia
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdomDeltaThresholdNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()