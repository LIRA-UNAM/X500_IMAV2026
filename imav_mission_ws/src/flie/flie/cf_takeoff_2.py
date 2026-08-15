import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import logging

import cflib.crtp 
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from flie.pose_publisher import CrazyfliePosePublisher

URI = 'radio://0/80/2M'

SM_DISCONNECTED = 0
SM_WAIT_TAKEOFF = 10
SM_FLYING = 20
SM_LANDING = 30 

#Velocidades
VELOCITY_LAND = 0.2
VELOCITY_EMER = 0.6

class CFHardwareNode(Node):
    def __init__(self):
        super().__init__('cf_hardware_node')

        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('pose_topic', 'hardware/pose_raw')

        self.create_subscription(Bool, 'hardware/start_takeoff', self.takeoff_cb, 10)
        self.create_subscription(Twist, 'hardware/cmd_vel_cf', self.vel_cb, 10)
        self.create_subscription(Bool, 'hardware/start_landing', self.land_cb, 10)

        self.create_subscription(Bool, 'emergency/stop', self.stop_cb, 10)

        self.takeoff_pub = self.create_publisher(Bool, 'hardware/takeoff_ready', 10)
        self.pose_publisher = CrazyfliePosePublisher(self, world_frame_id=self.get_parameter('world_frame_id').value, topic=self.get_parameter('pose_topic').value )

        self.state = SM_DISCONNECTED
        self.takeoff_requested = False
        self.land_requested = False
        self.stop_requested = False
        self.emergency_landing = False

        self.scf = None
        self.mc = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def takeoff_cb(self, msg: Bool):
        if msg.data and not self.takeoff_requested:
            self.takeoff_requested= True

    def vel_cb(self, msg: Twist):
        if self.mc is not None and self.state == SM_FLYING and not self.stop_requested:
            self.mc.start_linear_motion(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def land_cb(self, msg: Bool):
        if msg.data and not self.land_requested:
            self.land_requested = True

    def stop_cb(self, msg: Bool):
        if msg.data and not self.stop_requested:
            self.stop_requested = True

    def control_loop(self):
        if self.stop_requested and self.state not in (SM_DISCONNECTED, SM_LANDING):
            self.emergency_landing = True
            self.state = SM_LANDING

        if self.state == SM_DISCONNECTED:
            self.scf = SyncCrazyflie(URI)
            self.scf.open_link()

            self.pose_publisher.reset_estimator(self.scf)
            self.pose_publisher.start_logging(self.scf)

            self.state = SM_WAIT_TAKEOFF


        elif self.state == SM_WAIT_TAKEOFF:
            if self.takeoff_requested:
                self.mc = MotionCommander(self.scf)
                self.mc.take_off()

                rdy_msg = Bool()
                rdy_msg.data = True
                self.takeoff_pub.publish(rdy_msg)

                self.state = SM_FLYING

        elif self.state == SM_FLYING:
            if self.land_requested:
                self.state = SM_LANDING

        elif self.state == SM_LANDING:
            if self.mc:
                land_velocity = VELOCITY_EMER if self.emergency_landing else VELOCITY_LAND    
                self.mc.land(velocity=land_velocity)
                self.mc = None

                if self.scf:
                    self.scf.close_link()
                    self.scf = None


                self.emergency_landing = False
                self.timer.cancel()

    def destroy_node(self):
        if self.mc:
            self.mc.land()
            self.mc = None
        if self.scf:
            self.scf.close_link()
            self.scf = None
        super().destroy_node()

def main(args=None):
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_drive=False)
    rclpy.init(args=args)
    node = CFHardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
