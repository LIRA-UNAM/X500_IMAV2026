#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CFTranslatorNode(Node):
    def __init__(self):
        super().__init__('cf_translator_node')

        # Time
        self.current_twist = Twist()
        self.last_msg_time = time.time()
        
        # Listen cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        
        # Publish cmd_vel for crazyflie
        self.pub_hw_vel = self.create_publisher(Twist, 'hardware/cmd_vel_cf', 10)

        #Timer for publish
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info("Twist translate node ready")

    def cmd_vel_cb(self, msg: Twist):
        self.current_twist = msg
        self.last_msg_time = time.time()

    def timer_cb(self):
        hw_msg = Twist()
        if (time.time() - self.last_msg_time) > 0.5:
            hw_msg.linear.x = 0.0
            hw_msg.linear.y = 0.0
            hw_msg.linear.z = 0.0
            hw_msg.angular.z = 0.0
        else:
            hw_msg.linear.x = self.current_twist.linear.x
            hw_msg.linear.y = self.current_twist.linear.y
            hw_msg.linear.z = self.current_twist.linear.z
            hw_msg.angular.z = math.degrees(self.current_twist.angular.z)
            
        self.pub_hw_vel.publish(hw_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CFTranslatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
