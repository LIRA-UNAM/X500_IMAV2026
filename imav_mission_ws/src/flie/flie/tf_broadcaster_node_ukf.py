#!/usr/bin/env python3
"""
tf_broadcaster_node

Nodo ROS2 independiente -- NO toca cflib, NO abre conexión al Crazyflie.
Se suscribe a un PoseStamped (por defecto en 'hardware/pose_raw') y
republica esa información como una tf (world -> child_frame_id).

Al no depender de cflib, este mismo nodo sirve para cualquier
plataforma (Crazyflie, X500, etc.) mientras alguien publique un
PoseStamped en el tópico correcto -- solo cambia el parámetro
child_frame_id y, si hace falta, el tópico de entrada.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')

        self.declare_parameter('pose_topic', 'hardware/pose_raw')
        self.declare_parameter('child_frame_id', 'crazyflie')

        pose_topic = self.get_parameter('pose_topic').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)

        self.get_logger().info(
            f"tf_broadcaster_node listo. Escuchando '{pose_topic}', "
            f"publicando tf hacia child_frame_id='{self.child_frame_id}'"
        )

    def pose_cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # world_frame_id que vino en el mensaje
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

