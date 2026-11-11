import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import yaml
import os
from datetime import datetime

class YamlCoord(Node):
    def __init__(self):
        super().__init__('yaml_coord')

        #Suscription
        self.create_subscription(PoseArray, 'aruco/poses', self.pose_cb, 10)

        self.marker_data = {}

        #Ruta relativa para guardar el .yaml
        self.save_dir = os.path.expanduser('~/X500_IMAV2026/imav_mission_ws/src/vision/config')
        os.makedirs(self.save_dir, exist_ok=True)
        self.yaml_file = os.path.join(self.save_dir, 'coord_map.yaml')

        #Timer
        self.create_timer(2.0, self.save_yaml)
        self.get_logger().info('Nodo creado, esperando detección de arucos')

    def pose_cb(self, msg: PoseArray):
        for pose in msg.poses:
            marker_id = 5

            x = round(pose.position.x, 4)
            y = round(pose.position.y, 4)
            z = round(pose.position.z, 4)

            qx = round(pose.orientation.x, 4)
            qy = round(pose.orientation.y, 4)
            qz = round(pose.orientation.z, 4)
            qw = round(pose.orientation.w, 4)


            if marker_id not in self.marker_data:
                self.marker_data[marker_id] = {'x':x, 'y':y, 'z':z, 'qx':qx, 'qy':qy, 'qw':qw}
            else:
                prev = self.marker_data[marker_id]
                self.marker_data[marker_id] = {
                    'x': round(0.8 * prev['x'] + 0.2 * x, 4),
                    'y': round(0.8 * prev['y'] + 0.2 * y, 4),
                    'z': round(0.8 * prev['z'] + 0.2 * z, 4),
                    'qx': round(0.8 * prev['qx'] + 0.2 * qx, 4),
                    'qy': round(0.8 * prev['qy'] + 0.2 * qy, 4),
                    'qz': round(0.8 * prev['qz'] + 0.2 * qz, 4),
                    'qw': round(0.8 * prev['qw'] + 0.2 * qw, 4),
                }

    def save_yaml(self):
        if not self.marker_data:
            return

        yaml_content = {
            'reference_frame': 'camera_color_optical_frame', 
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'markers': {}
        }

        for m_id, data in self.marker_data.items():
            yaml_content['markers'][f'id_{m_id}'] = {
                'position_meters': [data['x'], data['y'], data['z']],
                'orientation_quaternion': [data['qx'], data['qy'], data['qz'], data['qw']]

            }

        with open(self.yaml_file, 'w') as f:
            yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'Coordenadas guardadas en {self.yaml_file}')

    def main(args=None):
        rclpy.init(args=args)
        node = YamlCoord()
        rclpy.spin(node)
        node.save_yaml()
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()