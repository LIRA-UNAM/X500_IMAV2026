import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
from tf_transformations import quaternion_multiply, quaternion_inverse

# Fixed rotations between PX4 and ROS conventions (q: x, y, z, w)
_Q_NED_TO_ENU = (0.70710678, 0.70710678, 0.0, 0.0)  # world: NED -> ENU
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)                # body:  FRD -> FLU


class OdomToTFNode(Node):

    def __init__(self):
        super().__init__('x500_odom_to_tf')

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.broadcaster_ = TransformBroadcaster(self)

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_cb,
            sub_qos)

        # Publish the TF at 50 Hz
        self.tf_timer_ = self.create_timer(1.0 / 50.0, self.tf_timer_callback)

        # Origin captured: only position
        self.origin_set_ = False
        self.origin_pos_ = (0.0, 0.0, 0.0)     # NED


        # Last computed pose
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        self.qx_ = 0.0
        self.qy_ = 0.0
        self.qz_ = 0.0
        self.qw_ = 1.0
        self.have_pose_ = False

        self.last_timestamp_us_ = 0
        
        self.get_logger().info("Odom To TF Node for X500 started.")

    #frame helpers

    @staticmethod
    def ned_to_enu_position(x_ned, y_ned, z_ned):
        return y_ned, x_ned, -z_ned

    #callbacks

    def odometry_cb(self, msg: VehicleOdometry):
        # msg.position -> [x, y, z] NED
        # msg.q        -> [w, x, y, z] FRD body w.r.t NED world  (PX4/uORB order)

        if any(math.isnan(v) for v in msg.position) or any(math.isnan(v) for v in msg.q):
            return  # EKF not initialized yet


        self.last_timestamp_us_ = msg.timestamp #store sensor timestamp

        # Reorder PX4's (w, x, y, z) into the ROS/tf_transformations (x, y, z, w)
        q_cur = (msg.q[1], msg.q[2], msg.q[3], msg.q[0])

        if not self.origin_set_:
            self.origin_pos_ = (msg.position[0], msg.position[1], msg.position[2])
            self.origin_set_ = True
            self.get_logger().info(
                "Origin captured at NED ({:.2f}, {:.2f}, {:.2f}). "
                "This pose is now (0,0,0) / identity in 'map'.".format(*self.origin_pos_))

        # Pose relative to the captured origin, still in NED/FRD
        rel_x = msg.position[0] - self.origin_pos_[0]
        rel_y = msg.position[1] - self.origin_pos_[1]
        rel_z = msg.position[2] - self.origin_pos_[2]
        self.x_, self.y_, self.z_ = self.ned_to_enu_position(rel_x, rel_y, rel_z)

        q_enu = quaternion_multiply(
            quaternion_multiply(_Q_NED_TO_ENU, q_cur), _Q_FRD_TO_FLU)
        self.qx_, self.qy_, self.qz_, self.qw_ = q_enu
 
        self.have_pose_ = True

    def tf_timer_callback(self):
        if not self.have_pose_:
            return

        t = TransformStamped()

        stamp_ns = self.last_timestamp_us_ * 1000
        t.header.stamp.sec     = int(stamp_ns // 1_000_000_000)
        t.header.stamp.nanosec = int(stamp_ns  % 1_000_000_000)

        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(self.x_)
        t.transform.translation.y = float(self.y_)
        t.transform.translation.z = float(self.z_)
 
        t.transform.rotation.x = float(self.qx_)
        t.transform.rotation.y = float(self.qy_)
        t.transform.rotation.z = float(self.qz_)
        t.transform.rotation.w = float(self.qw_)

        self.broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()