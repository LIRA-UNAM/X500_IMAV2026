import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor
)


class PX4FlowPrecision(Node):
    def __init__(self):
        super().__init__('px4_flow_precision')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.flow_cb, sub_qos)

        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz
        self.counter = 0

        # Posición actual
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0

        # Altura medida por el sensor de distancia
        self.current_distance = 0.0

        # Posición bloqueada al armar
        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        # Parámetros de vuelo
        self.target_altitude = 1.5   # Altura objetivo en metros
        self.target_z        = -1.5  # Mismo valor en NED para el setpoint de posición
        self.hold_duration   = 3.0   # hover seg
        #Forward
        self.forward_dis    = 2.0   #Distancia a la que se va a desplazar
        self.forward_tol    = 0.15  #Error que puede tener
        self. forward_hold  = 3.0   #Hold en nueva posición

        # Control de estados
        self.state            = "INIT"
        self.hold_start_time  = None    #Hold inicial
        self.hold_fwd_start   = None    #Hold forward
        self.stable_ticks     = 0
        self.stable_ticks_needed = 10  # 1 segundo a 10 Hz

    # ===================== CALLBACKS =====================

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg):
        self.current_distance = msg.current_distance

        if self.counter % 10 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | "
                f"Distancia: {self.current_distance:.4f} m"
            )

    # ===================== LOOP PRINCIPAL =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        # Modo Offboard
        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        # Setpoint base con NaN
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # Bloquear posición XY y yaw mientras está en tierra
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        # ===================== MÁQUINA DE ESTADOS =====================

        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # Modo OFFBOARD
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)  # Armar motores
                self.get_logger().info(
                    f"ARMADO - Ascendiendo a {self.target_altitude} m"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]

            # Usar sensor de distancia como fuente de altura (no current_z)
            error_alt = abs(self.current_distance - self.target_altitude)

            if error_alt < 0.40:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"target={self.target_altitude:.2f} m "
                    f"err={error_alt:.2f} m "
                    f"stable={self.stable_ticks}/{self.stable_ticks_needed}"
                )

            if self.stable_ticks >= self.stable_ticks_needed:
                self.state = "HOLD"
                self.get_logger().info(
                    f"HOLD POSITION - Estable en {self.current_distance:.2f} m "
                    f"(target={self.target_altitude:.2f} m, err={error_alt:.2f} m)"
                )

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]

            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_start_time
            ).nanoseconds / 1e9

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"HOLD {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m"
                )

            if elapsed >= self.hold_duration:
                self.forward_x = safe_x + self.forward_dis * math.cos(self.locked_yaw)
                self.forward_y = safe_y + self.forward_dis * math.sin(self.locked_yaw)
                self-self.state = "FORWARD"
                self.get_logger().info("Desplazandose 2.0 metros")
        
        elif self.state == "FORWARD":
            setpoint.position = [self.forward_x, self.forward_y, self-self.target_z]
            dist_target = math.sqrt(
                (self.current_x - self.forward_x) ** 2 + (self.current_y - self.forward_y) ** 2
            )
            
            if self.counter % 10 == 0:
                self.get_logger().info("MOVE FORWARD")

            if dist_target < self.forward_tol:
                self.state = "HOLD_FORWARD"
                self.get_logger().info("Llegó al destino")

        elif self.state == "HOLD_FORWARD":
            setpoint.position = [self.forward_x, self.forward_y, self-self.target_z]
            if self.hold_fwd_start is None:
                self.hold_fwd_start = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_fwd_start).nanoseconds / 1e9
            
            if self.counter % 10 == 0:
                self.get_logger().info("HOLD FORWARD comenando")

            if elapsed >= self.forward_hold:
                self.state = "LAND"
                self.get_logger().info("LAND, aterizando en el nuevo punto")

        
        elif self.state == "LAND":
            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]  # Descenso suave

            # Usar sensor de distancia para detectar aterrizaje
            if self.current_distance < 0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)  # Desarmar motores
                self.get_logger().info("LANDING COMPLETED - Motores desarmados")

        self.trajectory_pub.publish(setpoint)
        self.counter += 1

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PX4FlowPrecision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()