import rclpy
import math
from rclpy.node import Node 
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import DistanceSensor
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleAttitude


class PX4Move(Node):
    def __init__(self):
        super().__init__('px4_move')

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

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, sub_qos)
        self.attitude_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, sub_qos)
        self.flow_sub = self.create_subscription(DistanceSensor, '/fmu/out/distance_sensor', self.flow_cb, sub_qos)

        self.timer = self.create_timer(0.02, self.timer_cb)
        self.counter = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.current_distance = 0.0

        self.locked_x = None
        self.locked_y = None
        self.locked_yaw = None
        
        self.target_z = -1.2

        self.hold_time = 5.0
        self.TIMES = 0.0

        self.state = "INIT"
        self.ct = 0
        
    def local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
    
    def attitude_cb(self, msg):
        q = msg.q 
        siny= 2*(q[0]*q[3] + q[1] * q[2])
        cosy = 1 - 2*(q[2]*q[2] + q[3]*q[3])
        self.current_yaw = math.atan2(siny, cosy)
    
    def flow_cb(self, msg):
        self.current_distance = msg.current_distance
        if self.counter % 30 == 0:
            self.get_logger().info(f"Distancia: {self.current_distance:.2f}m")

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True   
        offboard.velocity = False  
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.yawspeed = float('nan')

        if self.state in ("INIT", "ARMING"):
            self.locked_x   = self.current_x
            self.locked_y   = self.current_y
            self.locked_yaw = self.current_yaw

        safe_x = self.locked_x   if self.locked_x   is not None else 0.0
        safe_y = self.locked_y   if self.locked_y   is not None else 0.0
        setpoint.yaw = self.locked_yaw if self.locked_yaw is not None else 0.0

        if self.state == "INIT":
            if self.counter > 100:
                self.send_cmd(176, param1=1.0, param2=6.0)
                self.state = "ARMING"
                self.counter = 0
        
        elif self.state == "ARMING":
            if self.counter > 150:
                self.send_cmd(400, param1=1.0)
                self.state = "TAKEOFF"
                self.counter = 0
        
        elif self.state == "TAKEOFF":
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.yaw = self.current_yaw

            error_alt = abs(self.current_distance + self.target_z)
            if error_alt < 0.20:
                self.ct +=1
            else:
                self.ct = 0

            if self.counter % 40 == 0:
                self.get_logger().info(
                    f"TAKEOFF | dist={self.current_distance:.2f} m "
                    f"err={error_alt:.2f}m" 
                    f"stable_ticks={self.ct}"
                )
            
            if self.ct == 100:
                self.TIMES = self.get_clock().now()
                self.state = "HOLD"
                self.ct = 0
                self.counter = 0
            
        elif self.state == "HOLD":
            setpoint.position = [safe_x, safe_y, self.target_z]
            setpoint.yaw = self.current_yaw

            elapsed = (self.get_clock().now() - self.TIMES).nanoseconds / 1e9

            if elapsed >= self.hold_time:
                self.get_logger().info(" HOLD por 5 SEGUNDOS")
                self.state = "LAND"

        elif self.state == "LAND":
            setpoint.position = [safe_x, safe_y, 0.0]
            setpoint.yaw = self.current_yaw

            if self.current_distance < 0.15:
                self.send_cmd(400, param1=0.0)
                self.get_logger().info("ATERRIZAJE COMPLETADO")
                self.state = "DONE"

        elif self.state == "DONE":
            self.timer.cancel()
            return

        
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
    node = PX4Move()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cerrando nodo")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


        
