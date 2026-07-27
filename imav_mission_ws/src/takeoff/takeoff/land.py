import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, VehicleLocalPosition, VehicleCommand, TrajectorySetpoint, VehicleControlMode

SM_WAIT_LAND = 0
SM_LANDING = 10
SM_DISARMED = 20

LOOP_RATE = 20
ALTITUDE_TOL = 0.15
WAIT_TIME = 5

class LandNode(Node):

    def land_ready_cb(self, msg: Bool):
        if msg.data and not self.land_flag:
            self.land_flag = True
            self.get_logger().info("Land_ready recibido")

    def local_pos_cb(self, msg: VehicleLocalPosition):
        # EKF2 Position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    def publish_offboard_hearbeat(self):
        ocm = OffboardControlMode()
        ocm.timestamp = self.get_clock().now().nanoseconds // 1000
        ocm.position = True
        ocm.velocity = False
        self.offboard_pub.publish(ocm)

    def send_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        now = self.get_clock().now().nanoseconds // 1000
    
        msg = VehicleCommand()
        msg.timestamp = now
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def setpoint_pub(self, x:float, y:float, z:float, yaw:float):
        now = self.get_clock().now().nanoseconds // 1000

        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [float(x), float(y), float(z)]
        sp.velocity = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float(yaw)
        sp.yawspeed = float('nan')
        self.trajectory_pub.publish(sp)

    def control_mode_cb(self, msg:VehicleControlMode):
        self.is_offboard = msg.flag_control_offboard_enabled
        self.is_armed = msg.flag_armed

    def __init__(self):
        super().__init__('land_node')
        self.get_logger().info("LANDING PROCCES")

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

        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_cb, sub_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, sub_qos)
        self.create_subscription(Bool, '/land_ready', self.land_ready_cb ,1)
    
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.land_flag = False
        self.is_armed = True
        self.mission_start = None


    def spin(self):
        self.get_logger().info("LANDING")
        state = SM_WAIT_LAND
        counter = 0

        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()

            if state == SM_WAIT_LAND:
                if self.land_flag and self.mission_start is None:
                    self.mission_start = self.get_clock().now()
                    self.get_logger().info("Land ready recibido")
                    

                if self.mission_start is not None:
                    self.publish_offboard_hearbeat()
                    elapsed = (now - self.mission_start).nanoseconds / 1e9
                    if elapsed >= WAIT_TIME:
                        state = SM_LANDING

            elif state == SM_LANDING:
                self.publish_offboard_hearbeat()
                self.setpoint_pub(self.current_x, self.current_y, 0.0, self.current_yaw)

                if abs(self.current_z) < ALTITUDE_TOL:
                    state = SM_DISARMED

            elif state == SM_DISARMED:
                if counter % LOOP_RATE == 0:
                    self.get_logger().info("Enviando comando de disarm")
                self.send_cmd(400, param1=0.0)
                if not self.is_armed:
                    self.get_logger().info("Drone desarmado")
                    break

            counter += 1
            self.get_clock().sleep_for(Duration(seconds=1.0/LOOP_RATE))


def main(args=None):
    rclpy.init(args=args)
    node = LandNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    







            

