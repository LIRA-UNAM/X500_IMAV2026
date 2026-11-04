import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from tf_transformations import quaternion_multiply, euler_from_quaternion

_Q_NED_TO_ENU = (0.70710678, 0.70710678, 0.0, 0.0)
_Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Threshold parameters
        self.declare_parameter('distance_threshold', 0.2)
        self.declare_parameter('yaw_threshold', 0.3)

        #Subscription to the optical flow data
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_cb,
            1
        )

        #Variables for previous values
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_yaw = None
        
        #Threshold
        umbral_dist = self.get_parameter('distance_threshold').value
        umbral_yaw = self.get_parameter('yaw_threshold').value
        self.get_logger().info(
            f"Delta node start.\n"
            f"Distance threshold: {umbral_dist} meters\n"
            f"Yaw threshold: {umbral_yaw} radians"
        )

    def odometry_cb(self, msg: VehicleOdometry):
        # Extract current data and convert to ENU
        curr_x = msg.position[1]
        curr_y = msg.position[0]
        curr_z = -msg.position[2]

        q_cur = (msg.q[1], msg.q[2], msg.q[3], msg.q[0]) 
        q_enu = quaternion_multiply(quaternion_multiply(_Q_NED_TO_ENU, q_cur), _Q_FRD_TO_FLU)
        _, _, curr_yaw = euler_from_quaternion(q_enu)

        # Initialize the reference point
        if self.prev_x is None:
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw
            self.get_logger().info(f"Starter point: X:{curr_x:.2f}, Y:{curr_y:.2f}, Z:{curr_z:.2f}, Yaw:{curr_yaw:.2f}")
            return

        # Calculate deltas
        delta_x = curr_x - self.prev_x
        delta_y = curr_y - self.prev_y
        delta_z = curr_z - self.prev_z
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        delta_yaw = curr_yaw - self.prev_yaw
        # Normalaize to a range [-pi, pi]
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
        abs_delta_yaw = abs(delta_yaw)

        # Reed args
        dist_threshold = self.get_parameter('distance_threshold').value
        yaw_threshold = self.get_parameter('yaw_threshold').value

        # Delta conditions
        if distance >= dist_threshold or abs_delta_yaw >= yaw_threshold:
            
            # Print data
            Trigger = "DISTANCE" if distance >= dist_threshold else "YAW"
            if distance >= dist_threshold and abs_delta_yaw >= yaw_threshold:
                Trigger = "Both"

            self.get_logger().info(
                f"\n[Update by: {Trigger} | Dist: {distance:.3f}m, YAW: {abs_delta_yaw:.3f}rad]\n"
                f"Deltas -> dx: {delta_x:.4f}m, dy: {delta_y:.4f}m, dz: {delta_z:.4f}m, dyaw: {delta_yaw:.4f}rad"
            )

            # Update the reference point
            self.prev_x = curr_x
            self.prev_y = curr_y
            self.prev_z = curr_z
            self.prev_yaw = curr_yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()