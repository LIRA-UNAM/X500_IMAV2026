import rclpy 
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    DistanceSensor
)

class PX4TrajectoryNode(Node):
    def __init__(self):
        super().__init__('px4_trajectory')

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
        self.offboard_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.flow_cb, sub_qos)

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.counter = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None

        self.target_z = -1.0
        self.hold_duration = 5.0
        self.hold_start_time = None
        self.point_x = 2.0
        
        self.state = "INIT"

    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def attitude_cb(self, msg):
        q = msg.q 
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw

    def flow_cb(self, msg):
        self.current_distance = msg.current_distance
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Calidad: {msg.signal_quality} | Distancia: {self.current_distance:.2f} m"
            )

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000
        
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = True

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now

        if self.state == "INIT" or self.state == "ARMING":
            self.locked_x = self.current_x
            self.locked_y = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x if self.locked_x is not None else 0.0
        safe_y = self.locked_y if self.locked_y is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        # STATE MACHINE
        if self.state == "INIT":
            if self.counter > 20:
                self.send_cmd(176, param1=1.0, param2=6.0) # Switch to Offboard
                self.state = "ARMING"

        elif self.state == "ARMING":
            if self.counter > 30:
                self.send_cmd(400, param1=1.0) # Arm motors
                self.get_logger().info(f"ARMED | Ascending to {self.target_z}m")
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]

            if abs(self.current_z - self.target_z) < 0.15:
                self.state = "HOLD"
                # FIX: Capture the exact time we enter HOLD to prevent TypeError
                self.hold_start_time = time.time()  
                self.get_logger().info(f"HOLD POSITION | {self.current_z}m")

        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]
            elapsed_time = time.time() - self.hold_start_time
            if elapsed_time >= self.hold_duration:
                self.state = "FORWARD"
                self.get_logger().info(f"Moving FORWARD to {self.point_x}m in X")

        elif self.state == "FORWARD":
            target_x = safe_x + self.point_x
            setpoint.position = [target_x, safe_y, self.target_z]
            if abs(self.current_x - target_x) < 0.15:
                self.state = "LANDING"
                self.get_logger().info(f"Reached target X: {self.current_x:.2f}m. Initiating Autoland.")
                # FIX: Send the native PX4 Land Command (ID 21)
                self.send_cmd(21) 

        elif self.state == "LANDING":
            setpoint.position = [self.current_x, self.current_y, float('nan')]            

        self.offboard_pub.publish(offboard)
        self.trajectory_pub.publish(setpoint)
        
        self.counter += 1

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
    node = PX4TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()