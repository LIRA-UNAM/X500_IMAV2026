import rclpy 
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point

SM_INIT = 0
SM_TAKEOFF = 10
SM_NAVIGATION = 20
SM_LANDING = 30
SM_DONE = 40
SM_STOP = 50

LOOP_RATE = 20

class StateMachine(Node):

    def ready_cb (self, msg: Bool):
        if msg.data:
            self.is_ready = True
            self.get_logger().info('Drone listooo')

    def arrived_cb(self, msg: Bool):
        if msg.data:
            self.has_arrived = True

    def stop_cb(self, msg:Bool):
        if msg.data and not self.stop_triggered:
            self.stop_triggered = True

            
    def __init__(self):
        super().__init__('state_machin_node')
        self.get_logger().info("State Machine Node started")

        self.ready_sub = self.create_subscription(Bool, 'hardware/takeoff_ready', self.ready_cb, 10)
        self.arrived_sub = self.create_subscription(Bool, 'navigator/arrived', self.arrived_cb, 10)
        self.stop_sub = self.create_subscription(Bool, 'emergency/stop', self.stop_cb, 10)

        self.takeoff_pub = self.create_publisher(Bool, 'hardware/start_takeoff', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.landing_pub = self.create_publisher(Bool, 'hardware/start_landing', 10)
        self.target_pub = self.create_publisher(Point, 'navigator/set_target', 10)
        self.nav_enable_pub = self.create_publisher(Bool, 'navigator/enable',10 )

        self.is_ready = False
        self.has_arrived = False
        self.stop_triggered = False

        self.command_sent = False

        self.waypoints = [
                    (0.0, -1.0, 1.2),
                    (2.0, -1.0, 1.2),
                    (1.0, -0.5, 1.0),
                    (2.0, -0.5, 1.0)
                ]
        self.current_waypoint_index = 0

    def spin(self):
        self.get_logger().info("State machine running")
        counter = 0
        state = SM_INIT

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.stop_triggered and state not in (SM_STOP, SM_DONE):
                state = SM_STOP

            if state == SM_INIT:
                if counter >= (LOOP_RATE * 2):
                    state = SM_TAKEOFF

            elif state == SM_TAKEOFF:
                if not self.command_sent:
                    self.takeoff_pub.publish(Bool(data=True))
                    self.command_sent = True

                if self.is_ready:
                    state = SM_NAVIGATION
                    self.command_sent = False

            elif state == SM_NAVIGATION:
                if not self.command_sent:
                    x, y, z = self.waypoints[self.current_waypoint_index]

                    self.nav_enable_pub.publish(Bool(data=True))

                    pto = Point()
                    pto.x, pto.y, pto.z = float(x), float(y), float(z)

                    self.target_pub.publish(pto)

                    self.has_arrived = False
                    self.command_sent = True

                if self.has_arrived:
                    self.current_waypoint_index += 1
                    self.command_sent = False

                    if self.current_waypoint_index >= len(self.waypoints):
                        state = SM_LANDING

            elif state == SM_LANDING:
                if not self.command_sent:
                    self.landing_pub.publish(Bool(data=True))
                    self.command_sent = True
                    state = SM_DONE

            elif state == SM_DONE:
                break

            elif state == SM_STOP: 
                if not self.command_sent:
                    self.nav_enable_pub.publish(Bool(data=False))
                    self.command_sent = True
                break

            counter += 1
            self.get_clock().sleep_for(Duration(seconds=1.0/LOOP_RATE))

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()