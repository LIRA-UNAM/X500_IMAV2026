import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleControlMode
)

class PX4FlowPrecision(Node):
    def __init__(self):
        super().__init__('px4_flow_precision')

        # QoS configuration for PX4 uXRCE-DDS
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_pos_cb,
            sub_qos)
        
        # self.attitude_sub = self.create_subscription(
        #     VehicleAttitude,
        #     '/fmu/out/vehicle_attitude',
        #     self.attitude_cb,
        #     sub_qos)

        self.control_mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.control_mode_cb,
            sub_qos)

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos)
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            pub_qos)
        
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            pub_qos)

        

        # Main Loop at 20 Hz Recommended for PX4 Offboard
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.counter = 0

        # State Variables
        self.is_offboard = False
        self.is_armed = False
        
        # Position Data (EKF2 Filtered)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        # Locked Setpoints
        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = 0.0

        # Mission Parameters
        self.target_altitude = 1.2           
        self.target_z = -self.target_altitude # NED coordinates: Z is negative
        self.hold_duration = 5.0             # Hover time in seconds

        # State Machine Control
        self.state = "INIT"
        self.hold_start_time = None
        self.stable_ticks = 0
        self.stable_ticks_needed = 20        # 1 second at 20 Hz

    # ===================== CALLBACKS =====================

    def local_pos_cb(self, msg):
        # EKF2 Position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def control_mode_cb(self, msg):
        # Monitor the actual state of the Pixhawk 6x
        self.is_offboard = msg.flag_control_offboard_enabled
        self.is_armed = msg.flag_armed

    # ===================== MAIN CONTROL LOOP =====================

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000
        
        # OffboardControlMode
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        self.offboard_pub.publish(offboard)

        #Base Trajectory Setpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp    = now
        setpoint.position     = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity     = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk         = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw          = float('nan')
        setpoint.yawspeed     = float('nan')

        # Lock starting position  XY before takeoff
        if self.state in ("INIT", "REQUESTING_OFFBOARD"):
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw

        # ===================== STATE MACHINE =====================

        if self.state == "INIT":
            if self.counter > 20: 
                self.get_logger().info("Solicitando Modo Offboard y Armado")
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.send_cmd(400, param1=1.0)         
                self.state = "WAITING_CONFIRMATION"

        elif self.state == "WAITING_CONFIRMATION":
            # Only proceed if Pixhawk confirms
            if self.is_offboard and self.is_armed:
                self.get_logger().info(f"CONFIRMADO. Despegando a {self.target_altitude}m")
                self.state = "TAKEOFF"
            elif self.counter % 20 == 0:
                self.get_logger().info("Esperando confirmación de Armado y Offboard de la Pixhawk...")
                self.send_cmd(176, param1=1.0, param2=6.0) 
                self.send_cmd(400, param1=1.0)

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]
            
            error_alt = abs(self.current_z - self.target_z)

            if error_alt < 0.20: # 20cm tolerance
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0

            if self.counter % 20 == 0:
                alt_actual = abs(self.current_z)
                self.get_logger().info(f"TAKEOFF | Altura: {alt_actual:.2f}m / Objetivo: {self.target_altitude:.2f}m | Error: {error_alt:.2f}m")

            if self.stable_ticks >= self.stable_ticks_needed:
                self.get_logger().info("ALTURA ALCANZADA. Iniciando Hold.")
                self.state = "HOLD"

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]

            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9

            if self.counter % 20 == 0:
                self.get_logger().info(f"HOLD | Tiempo: {elapsed:.1f}s / {self.hold_duration}s")

            if elapsed >= self.hold_duration:
                self.get_logger().info("Misión completada. Iniciando Auto-Aterrizaje.")
                self.send_cmd(21) # VEHICLE_CMD_NAV_LAND
                self.state = "LANDING"

        elif self.state == "LANDING":
            if self.counter % 20 == 0:
                self.get_logger().info(f"ATERRIZANDO... Altura actual: {abs(self.current_z):.2f}m")

            if not self.is_armed:
                self.get_logger().info("VEHÍCULO DESARMADO.")
                self.state = "LANDED"

        elif self.state == "LANDED":
            pass

        if self.state not in ["LANDING", "LANDED"]:
            self.trajectory_pub.publish(setpoint)
            
        self.counter += 1

    # ===================== COMMAND HELPER =====================

    def send_cmd(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
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