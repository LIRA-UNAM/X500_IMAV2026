#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
import time
import threading

class CFBrainNode(Node):
    def __init__(self):
        super().__init__('cf_brain_node')
        self.takeoff_pub = self.create_publisher(Bool, 'hardware/start_takeoff', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.landing_pub = self.create_publisher(Bool, 'hardware/start_landing', 10)
        self.target_pub = self.create_publisher(Point, 'navigator/set_target', 10)
        self.nav_enable_pub = self.create_publisher(Bool, 'navigator/enable', 10)

        self.ready_sub = self.create_subscription(Bool, 'hardware/takeoff_ready', self.ready_cb, 10)
        self.arrived_sub = self.create_subscription(Bool, 'navigator/arrived', self.arrived_cb, 10)
        
        self.is_ready = False
        self.has_arrived = False
        self.get_logger().info("Starting Brain Node")

    def ready_cb(self, msg: Bool):
        if msg.data:
            self.is_ready = True
            self.get_logger().info("Drone ready")

    def arrived_cb(self, msg: Bool):
        if msg.data:
            self.has_arrived = True

    def toggle_navigator(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.nav_enable_pub.publish(msg)
        time.sleep(0.1)

    def publish_velocity_for_duration(self, twist_msg: Twist, duration: float):
        rate = self.create_rate(10) # 10 Hz (cada 0.1s)
        start_time = self.get_clock().now().nanoseconds / 1e9
        
        while rclpy.ok():
            current_time = self.get_clock().now().nanoseconds / 1e9
            if (current_time - start_time) >= duration:
                break
                
            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()

    def publish_coordinates(self, x, y, z):
        self.toggle_navigator(True)
        self.has_arrived = False
        
        pto = Point()
        pto.x = float(x)
        pto.y = float(y)
        pto.z = float(z)
        
        self.get_logger().info(f"Flyig to X:{x}, Y:{y}, Z:{z}")
        self.target_pub.publish(pto)
        
        while not self.has_arrived and rclpy.ok():
            time.sleep(0.1)
            
        self.get_logger().info("Drone ready in coordinate X:{x}, Y:{y}, Z:{z}")
        time.sleep(1.0)

    def run_sequence(self):
        time.sleep(2.0)

        # 1. Takeoff
        self.get_logger().info("Sending take off")
        msg = Bool()
        msg.data = True
        self.takeoff_pub.publish(msg)
        while not self.is_ready and rclpy.ok():
            time.sleep(0.5)
        time.sleep(1.0)

        # # 2. Backward 
        # self.get_logger().info("Moving Backward at 0.2 m/s...")
        # vel = Twist()
        # vel.linear.x = -0.2
        # self.publish_velocity_for_duration(vel, 1.0)

        # # 3. Stop (Frenar y mantener 0.0 durante 1.0 segundo)
        # self.get_logger().info("Stopping...")
        # vel = Twist() # Todo en 0.0
        # self.publish_velocity_for_duration(vel, 1.0)
        
        # 4. Choords
        self.publish_coordinates(2.0, -1.0, 1.0)
        self.publish_coordinates(1.0, -0.5, 0.5)
        
        # # 5. Turn left (Girar a 1.5 rad/s durante 1.0 segundo)
        # self.get_logger().info("Turning left 90°...")
        # vel = Twist()
        # vel.angular.z = 1.5
        # self.publish_velocity_for_duration(vel, 1.0) 
        
        # # 6. Stop (Frenar y mantener 0.0 durante 1.0 segundo)
        # self.get_logger().info("Stopping...")
        # vel = Twist()
        # self.publish_velocity_for_duration(vel, 1.0)

        # 7. Land
        self.get_logger().info("Sending land")
        land_msg = Bool()
        land_msg.data = True
        self.landing_pub.publish(land_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = CFBrainNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    node.run_sequence()
    spin_thread.join()

if __name__ == '__main__':
    main()