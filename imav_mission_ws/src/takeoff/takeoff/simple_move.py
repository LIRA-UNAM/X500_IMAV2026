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
    DistanceSensor,
)


class PX4FlowPrecision(Node):
    def __init__(self):
        super().__init__('px4_flow_precision')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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

        # Estado estimado del vehículo
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0

        self.xy_valid = False

        self.current_distance = 0.0
        self.flow_quality     = 0

        self.locked_x   = None
        self.locked_y   = None
        self.locked_yaw = None

        self.forward_target_x  = None
        self.forward_target_y  = None
        self.forward_distance  = 2.0  # metros

        # Parámetros de vuelo 
        self.target_altitude = 1.2   # Altura objetivo en metros (sobre el suelo)
        self.target_z        = -1.2  # Mismo valor en NED (Z negativo = arriba)
        self.hold_duration   = 3.0   # segundos de hover en HOLD y HOLD_FWD

        # Control de estados
        self.state               = "INIT"
        self.hold_start_time     = None
        self.hold_fwd_start_time = None
        self.stable_ticks        = 0
        self.stable_ticks_needed = 10  # 1 segundo a 10 Hz

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.xy_valid = msg.xy_valid

    def attitude_cb(self, msg: VehicleAttitude):
        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def flow_cb(self, msg: DistanceSensor):
        self.current_distance = msg.current_distance
        self.flow_quality     = msg.signal_quality
        if self.counter % 10 == 0:
            self.get_logger().info(
                f"[SENSOR] calidad={self.flow_quality:3d} | "
                f"dist={self.current_distance:.3f} m | "
                f"xy_valid={self.xy_valid}"
            )

    # =========================================================================
    # LOOP PRINCIPAL
    # =========================================================================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp    = now
        offboard.position     = True
        offboard.velocity     = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed     = float('nan')

        # Bloqueo de posición 
        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x   = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y   = self.locked_y   if self.locked_y   is not None else 0.0
        safe_yaw = self.locked_yaw if self.locked_yaw is not None else 0.0
        setpoint.yaw = safe_yaw

        # =========================================================================
        # MÁQUINA DE ESTADOS
        # =========================================================================

        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0)  # MAV_CMD: modo OFFBOARD
                self.state = "ARMING"
                self.get_logger().info("INIT→ARMING: modo Offboard solicitado")

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0)  # MAV_CMD_COMPONENT_ARM_DISARM
                self.get_logger().info(
                    f"ARMING→TAKEOFF: motores armados, "
                    f"subiendo a {self.target_altitude} m"
                )
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]
            error_alt = abs(self.current_distance - self.target_altitude)

            if error_alt < 0.40:
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0 

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[TAKEOFF] dist={self.current_distance:.2f} m | "
                    f"err={error_alt:.2f} m | "
                    f"stable={self.stable_ticks}/{self.stable_ticks_needed} | "
                    f"xy_valid={self.xy_valid}"
                )

            if self.stable_ticks >= self.stable_ticks_needed:
                self.state = "HOLD"
                self.hold_start_time = None
                self.get_logger().info(
                    f"TAKEOFF→HOLD: estable en {self.current_distance:.2f} m"
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
                    f"[HOLD] {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m | "
                    f"xy_valid={self.xy_valid}"
                )

            if elapsed >= self.hold_duration:
                if self.xy_valid:
                    self.forward_target_x = (
                        self.current_x + self.forward_distance * math.cos(safe_yaw)
                    )
                    self.forward_target_y = (
                        self.current_y + self.forward_distance * math.sin(safe_yaw)
                    )
                    self.state = "FORWARD"
                    self.get_logger().info(
                        f"HOLD→FORWARD: destino NED "
                        f"({self.forward_target_x:.2f}, {self.forward_target_y:.2f}) | "
                        f"yaw={math.degrees(safe_yaw):.1f}°"
                    )
                else:
                    self.get_logger().warn(
                        "HOLD: xy_valid=False → no se puede avanzar. "
                        "Verificar calidad del H-Flow y parámetros EKF2_OF_CTRL."
                    )
                    self.hold_start_time = None

        elif self.state == "FORWARD":
            setpoint.position = [
                self.forward_target_x,
                self.forward_target_y,
                self.target_z,
            ]

            dx = self.forward_target_x - self.current_x
            dy = self.forward_target_y - self.current_y
            dist_to_target = math.sqrt(dx * dx + dy * dy)

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[FORWARD] dist_restante={dist_to_target:.2f} m | "
                    f"pos=({self.current_x:.2f}, {self.current_y:.2f}) | "
                    f"xy_valid={self.xy_valid}"
                )

            if not self.xy_valid and self.counter % 10 == 0:
                self.get_logger().warn(
                    "[FORWARD] xy_valid=False durante movimiento. "
                    "Manteniendo setpoint, esperando recuperación del sensor."
                )

            if dist_to_target < 0.25:
                self.state = "HOLD_FWD"
                self.hold_fwd_start_time = None
                self.get_logger().info(
                    f"FORWARD→HOLD_FWD: llegado, error residual={dist_to_target:.2f} m"
                )

        elif self.state == "HOLD_FWD":
            setpoint.position = [
                self.forward_target_x,
                self.forward_target_y,
                self.target_z,
            ]

            if self.hold_fwd_start_time is None:
                self.hold_fwd_start_time = self.get_clock().now()

            elapsed = (
                self.get_clock().now() - self.hold_fwd_start_time
            ).nanoseconds / 1e9

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[HOLD_FWD] {elapsed:.1f}s / {self.hold_duration}s | "
                    f"dist={self.current_distance:.2f} m | "
                    f"xy_valid={self.xy_valid}"
                )

            if elapsed >= self.hold_duration:
                self.state = "LAND"
                self.get_logger().info("HOLD_FWD→LAND: iniciando aterrizaje")

        elif self.state == "LAND":
            land_x = self.forward_target_x if self.forward_target_x is not None else safe_x
            land_y = self.forward_target_y if self.forward_target_y is not None else safe_y

            setpoint.position = [land_x, land_y, 0.0]
            setpoint.velocity = [float('nan'), float('nan'), 0.4]  # m/s descenso suave

            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"[LAND] dist={self.current_distance:.3f} m | "
                    f"pos=({self.current_x:.2f}, {self.current_y:.2f})"
                )

            # El rangefinder del H-Flow detecta que tocamos el suelo
            if self.current_distance < 0.15:
                self.state = "LANDED"
                self.send_cmd(400, param1=0.0)  # MAV_CMD_COMPONENT_ARM_DISARM
                self.get_logger().info("LAND→LANDED: suelo detectado, motores desarmados")

        elif self.state == "LANDED":
            pass

        if self.state != "LANDED":
            self.trajectory_pub.publish(setpoint)

        self.counter += 1

    # =========================================================================
    # UTILIDADES
    # =========================================================================

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
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