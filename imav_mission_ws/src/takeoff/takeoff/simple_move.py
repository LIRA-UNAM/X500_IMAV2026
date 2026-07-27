import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleLocalPosition,
)

#Constantes para la SM
SM_WAIT_READY = 0
SM_MOVING = 10
SM_HOVER = 20

# Parametros globales (variables que se pueden cambiar)
LOOP_RATE = 20
MOVE_SPEED = 0.3 
MOVE_DURATION = 5.0

class MissionNode(Node):

    def takeoff_ready(self, msg: Bool):
        if msg.data and not self.takeoff_ready_flag:
            self.takeoff_ready_flag = True
            self.get_logger().info("Takeoff_ready recibido.")

    def local_pos_cb(self, msg: VehicleLocalPosition):
        # EKF2 Position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def publish_offboard_hearbeat(self):
        """
        Funcion que mantiene al drone comunicando el modo offboard y no pierda señal
        o caiga en un failsafe.
        """
        ocm = OffboardControlMode()
        ocm.timestamp = self.get_clock().now().nanoseconds // 1000
        ocm.position = False
        ocm.velocity = True #Se le indica que se va a controlar la velocidad
        self.offboard_pub.publish(ocm)

    def publish_cmd_vel(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw_rate: float = 0.0):
        """"
        Funcion que se encarga de publicar la velocidad deseada y en donde cmd_vel_translate.py
        captura el mensaje y lo traduce a NED para ser un SetPoint de PX4.
        """
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self.cmd_vel_pub.publish(msg)

    # Constructor
    def __init__(self):
        super().__init__('simple_move_node')
        self.get_logger().info("INITIALIZING SIMPLE MOVE NODE")

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

        #Suscriptor
        self.create_subscription(
            Bool, 
            '/hardware/takeoff_ready',
            self.takeoff_ready,
            1
        )
        
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_pos_cb,
            sub_qos
        )

        #Publicador
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            pub_qos
        )
        
        self.release_hold_pub = self.create_publisher(
            Bool,
            '/hardware/release_hold',
            1
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self.land_flag_pub = self.create_publisher(Bool, '/land_ready', 1)

        # Position Data (EKF2 Filtered)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.takeoff_ready_flag = False
        self.mission_start = None
        self.land_flag = False

    def spin(self):
        self.get_logger().info("SIMPLE MOVE NODE IS RUNNING")
        counter = 0
        state = SM_WAIT_READY

        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()

            if state == SM_WAIT_READY:
                if self.takeoff_ready_flag:
                    self.publish_offboard_hearbeat()
                    self.release_hold_pub.publish(Bool(data=True))
                    self.get_logger().info("RELEASE HOLD SENT")
                    self.mission_start = self.get_clock().now()
                    state = SM_MOVING

            elif state == SM_MOVING:
                self.publish_offboard_hearbeat()
                self.publish_cmd_vel(vx=MOVE_SPEED)
                elapsed = (now - self.mission_start).nanoseconds / 1e9

                if counter % LOOP_RATE == 0:
                    self.get_logger().info("MOVING FORWARD: %.2f seconds elapsed" % elapsed)


                if elapsed >= MOVE_DURATION:
                    self.publish_cmd_vel()
                    self.get_logger().info("MOVE DURATION REACHED, HOVERING")
                    state = SM_HOVER

            elif state == SM_HOVER:
                self.publish_offboard_hearbeat()
                self.publish_cmd_vel()

                if not self.land_flag:
                    self.land_flag = True
                    self.land_flag_pub.publish(Bool(data=True))
                    self.get_logger().info("Land flag enviado, cediendo control")
                    break
                    
                if counter % LOOP_RATE == 0:
                    self.get_logger().info("HOVERING AT POSITION: x=%.2f, y=%.2f, z=%.2f" 
                                           % (self.current_x, self.current_y, self.current_z))
            
            counter +=1
            self.get_clock().sleep_for(Duration(seconds=1.0/LOOP_RATE))

def main(args = None):
    rclpy.init(args=args)
    node = MissionNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()