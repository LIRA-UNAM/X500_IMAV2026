#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import logging
import time
import threading

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from flie.pose_publisher import CrazyfliePosePublisher

URI = 'radio://0/80/2M'

class CFHardwareNode(Node):
    def __init__(self):
        super().__init__('cf_hardware_node')
        self.takeoff_cmd = False
        self.land_cmd = False
        self.mc = None

        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('pose_topic', 'hardware/pose_raw')
        
        #Subscribers
        self.create_subscription(Bool, 'hardware/start_takeoff', self.takeoff_cb, 10)
        self.create_subscription(Twist, 'hardware/cmd_vel_cf', self.vel_cb, 10)
        self.create_subscription(Bool, 'hardware/start_landing', self.land_cb, 10)
        
        # Publishers
        self.takeoff_rdy_pub = self.create_publisher(Bool, 'hardware/takeoff_ready', 10)
        self.pose_publisher = CrazyfliePosePublisher(self, world_frame_id=self.get_parameter('world_frame_id').value, topic=self.get_parameter('pose_topic').value,)
        self.get_logger().info("Hardware node start. Waiting conection...")

    def takeoff_cb(self, msg: Bool):
        if msg.data and not self.takeoff_cmd:
            self.takeoff_cmd = True
            self.get_logger().info("Received take off")

    def vel_cb(self, msg: Twist):
        if self.mc is not None and self.takeoff_cmd and not self.land_cmd:
            self.mc.start_linear_motion(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def land_cb(self, msg: Bool):
        if msg.data and not self.land_cmd:
            self.land_cmd = True
            self.get_logger().info("Received land")
            self.get_logger().info("Landing")

def main(args=None):
    rclpy.init(args=args)
    node = CFHardwareNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    try:
        with SyncCrazyflie(URI) as scf:
            node.get_logger().info("Conected. Waiting sign for /hardware/start_takeoff...")
            
            #Pose
            node.pose_publisher.reset_estimator(scf)
            node.pose_publisher.start_logging(scf)

            # Wait takeoff
            while not node.takeoff_cmd and rclpy.ok():
                time.sleep(0.1)

            if rclpy.ok():
                with MotionCommander(scf) as mc:
                    node.get_logger().info('Taking off')
                    node.mc = mc
                    time.sleep(1.0)
                    
                    # Sent ready
                    rdy_msg = Bool()
                    rdy_msg.data = True
                    node.takeoff_rdy_pub.publish(rdy_msg)
                    
                    # Flying
                    while rclpy.ok() and not node.land_cmd:
                        time.sleep(0.1)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.mc = None
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
