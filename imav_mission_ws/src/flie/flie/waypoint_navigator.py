#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Point
import math

class CFWaypointNavigator(Node):
    def __init__(self):
        super().__init__('cf_waypoint_navigator')
        
        # Inicial cordinate
        self.target = Point()
        self.target.x = 0.0
        self.target.y = 0.0
        self.target.z = 0.4
        
        # Actual position
        self.current_pose = Point()
        self.has_pose = False
        self.target_reached = False

        #
        self.is_enabled = True
        
        # Control parameters
        self.kp_xy = 0.5
        self.kp_z = 0.5
        self.max_vel = 0.5 # Max velocity m/s
        self.tolerance = 0.05 #Tolerance distance m
        
        # Suscriptores y Publicadores
        self.create_subscription(PoseStamped, 'hardware/pose_raw', self.pose_cb, 10)
        self.create_subscription(Point, 'navigator/set_target', self.target_cb, 10)
        self.create_subscription(Bool, 'navigator/enable', self.enable_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arrived_pub = self.create_publisher(Bool, 'navigator/arrived', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop_cb)
        
        self.get_logger().info("Waypoint navigator start. Waitting position...")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg.pose.position
        self.has_pose = True

    def target_cb(self, msg: Point):
        self.target.x = msg.x
        self.target.y = msg.y
        self.target.z = msg.z
        self.target_reached = False 
        self.get_logger().info(f"New destiny: X:{msg.x}, Y:{msg.y}, Z:{msg.z}")

    def enable_cb(self, msg: Bool):
        self.is_enabled = msg.data
        estado = "ACTIVATED" if self.is_enabled else "DEACTIVATED"
        self.get_logger().info(f"The navigate is {estado}.")

    def control_loop_cb(self):
        if not self.has_pose:
            return 
        if not self.is_enabled:
            return

        # 1. Calculate the error on each axis
        error_x = self.target.x - self.current_pose.x
        error_y = self.target.y - self.current_pose.y
        error_z = self.target.z - self.current_pose.z
        
        # Total Euclidean distance
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        msg = Twist()
        
        if distance < self.tolerance:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            if not self.target_reached:
                self.target_reached = True
                arr_msg = Bool()
                arr_msg.data = True
                self.arrived_pub.publish(arr_msg)
        else:
            # 2. Apply the control P: Velocity = Kp * Error
            vel_x = self.kp_xy * error_x
            vel_y = self.kp_xy * error_y
            vel_z = self.kp_z * error_z
            
            # 3. Velocity limmiter
            msg.linear.x = max(min(vel_x, self.max_vel), -self.max_vel)
            msg.linear.y = max(min(vel_y, self.max_vel), -self.max_vel)
            msg.linear.z = max(min(vel_z, self.max_vel), -self.max_vel)
            
            #Drone pointing forward
            msg.angular.z = 0.0 

        # 4. Sent instruction to translate node
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CFWaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
