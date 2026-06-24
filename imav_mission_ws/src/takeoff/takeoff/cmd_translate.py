import rclpy 
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition 

class CmdVelTranslateNode(Node):

    def __init__(self):
        super().__init__('cmd_translate')
        self.get_logger().info("TRANSLATING CMD_VEL TO PX4")

        pub_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, 
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        sub_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )
        
        self.px4_pub = self.create_publisher(Float32MultiArray, 'px4_cmd_vel', pub_qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, sub_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, sub_qos)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ang_vx = 0.0
        self.ang_vy = 0.0
        self.ang_vz = 0.0

        self.heading = 0.0

    def cmd_vel_callback(self, msg: Twist):
        #BODY
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z

        self.ang_vx = msg.angular.x
        self.ang_vy = msg.angular.y
        self.ang_vz = msg.angular.z

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.heading = msg.heading

        cos = np.cos(self.heading)
        sen = np.sin(self.heading)

        vx_ned = cos * self.vx - sen * self.vy
        vy_ned = sen * self.vx + cos * self.vy
        vz_ned = -self.vz
        
        msg1 = Float32MultiArray()
        msg1.data = [vx_ned, vy_ned, vz_ned]

        self.px4_pub.publish(msg1)


def main(args = None):
    rclpy.init(args=args)
    node = CmdVelTranslateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

