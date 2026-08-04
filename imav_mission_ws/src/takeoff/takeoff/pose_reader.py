import rclpy
from rclpy.node import Node
from rclpy.node import time

from tf2_ros import Buffer, TransformListenerm, LookupException, ExtrapolationException
from tf_transformations import euler_from_quaternion


class PoseReaderNode(Node):
    """
    ROS2 node that reads the pose of a robot from the TF2 transform tree.
    Prints the pose (meters and degrees) of the robot in the world frame.
    """

    READ_RATE = 10

    def __init__(self):
        super().__init__('pose_reader_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('drone_frame', 'base_link')
        
        self.map_frame_ = self.get_parameter('map_frame').value
        self.drone_frame_ = self.get_parameter('drone_frame').value

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        self.create_timer(1.0 / self.READ_RATE, self.timer_callback)
        
        self.get_logger().info("POSE READER NODE INITIALIZED. Reading TF '{}' -> '{}'".format(self.map_frame_, self.drone_frame_))
        
        
    def timer_callback(self):
        try:
            tf = self.tf_buffer_.lookup_transform(self.map_frame_, self.drone_frame_, Time())
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn("TF not available yet: {}".format(e), throttle_duration_sec=2.0)
            return
    
    #Position in meters
    x = tf.transform.translation.x
    y = tf.transform.translation.y
    z = tf.transform.translation.z
    
    #Orientation in degrees
    q = tf.transform.rotation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll_deg = roll * 180.0 / 3.141592653589793
    pitch_deg = pitch * 180.0 / 3.141592653589793
    yaw_deg = yaw * 180.0 / 3.141592653589793
    
    self.get_logger().info(
        "Pose: Position (x={:.2f}, y={:.2f}, z={:.2f}) meters \n"
        "Orientation (roll={:.2f}, pitch={:.2f}, yaw={:.2f}) degrees".format(
            x, y, z, roll_deg, pitch_deg, yaw_deg
        )
    )

def main(args=None):
    rclpy.init(args=args)
    node = PoseReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()