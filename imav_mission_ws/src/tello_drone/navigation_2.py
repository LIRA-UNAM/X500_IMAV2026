import math
import time
from djitellopy import Tello

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class Navigation:
    MIN = 20
    MAX = 500

    def __init__(self, tello: Tello):
        self.tello = tello
        self.tello.connect()
        bateria = self.tello.get_battery()
        print(f"Batería: {bateria}%")

        self.x_b = 0
        self.y_b = 0
        self.current_z = 0
        self.current_yaw = 0

        # Initialize ROS 2 Node
        rclpy.init()
        self.node = rclpy.create_node('tello_visualizer')
        self.pose_pub = self.node.create_publisher(PoseStamped, '/tello_pose', 10)
        
        # Publish initial position
        self.publish_pose()

    def publish_pose(self):
        """Converts local variables to a ROS 2 Pose message and publishes it."""
        msg = PoseStamped()
        
        # Standard RViz fixed frame
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        # Convert cm to meters for ROS
        msg.pose.position.x = self.x_b / 100.0
        msg.pose.position.y = self.y_b / 100.0
        msg.pose.position.z = self.current_z / 100.0

        # Convert Yaw (degrees) to Quaternion for RViz orientation
        rad = math.radians(self.current_yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(rad / 2.0)
        msg.pose.orientation.w = math.cos(rad / 2.0)

        self.pose_pub.publish(msg)
        # Process ROS callbacks
        rclpy.spin_once(self.node, timeout_sec=0.01)

    # --- Your existing movement functions ---
    
    def rotation(self, angle: int):
        if angle > 0: 
            self.tello.rotate_clockwise(angle)
        if angle < 0:
            self.tello.rotate_counter_clockwise(abs(angle))
        
        self.current_yaw = (self.current_yaw + angle) % 360
        time.sleep(3.0)
        
        # Update RViz after rotation
        self.publish_pose()

    def coord(self, x: float, y: float, z: float):
        self.tx = int(round(x * 100))
        self.ty = int(round(y * 100))
        self.tz = int(round(z * 100))

        self.current_z = self.tello.get_height()

        w_dx = self.tx - self.x_b
        w_dy = self.ty - self.y_b 
        dz = self.tz - self.current_z

        if dz != 0:
            if self.MIN <= abs(dz) <= self.MAX:
                if dz > 0:
                    self.tello.move_up(dz)
                elif dz < 0:
                    self.tello.move_down(abs(dz))
                # Optional: self.publish_pose() here if you want to see the Z step before XY

        rad = math.radians(self.current_yaw)
        local_dx = int(round(w_dx * math.cos(rad) + w_dy * math.sin(rad)))
        local_dy = int(round(-w_dx * math.sin(rad) + w_dy * math.cos(rad)))

        if local_dx != 0:
            if self.MIN <= abs(local_dx) <= self.MAX:
                if local_dx > 0:
                    self.tello.move_forward(local_dx)
                elif local_dx < 0:
                    self.tello.move_back(abs(local_dx))
                self.x_b = self.tx
                    
        if local_dy != 0:
            if self.MIN <= abs(local_dy) <= self.MAX:
                if local_dy > 0:
                    self.tello.move_right(local_dy)
                elif local_dy < 0:
                    self.tello.move_left(abs(local_dy))
                self.y_b = self.ty

        # Update RViz after the translation finishes
        self.publish_pose()

    def land(self):
        self.tello.land()
        time.sleep(2)
        # Shut down ROS safely
        self.node.destroy_node()
        rclpy.shutdown()
        
    def takeoff(self):
        self.tello.takeoff()
        time.sleep(2)