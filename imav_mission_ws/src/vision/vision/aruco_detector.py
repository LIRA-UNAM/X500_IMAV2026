#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parámetros configurables
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_1000')
        self.declare_parameter('marker_size_m', 0.13)  # tamaño real del marcador [m]

        img_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        caminfo_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_size = float(self.get_parameter('marker_size_m').get_parameter_value().double_value)
        dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value

        # Diccionario ArUco
        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f'Diccionario {dict_name} no válido, usando DICT_5X5_1000')
            dict_id = cv2.aruco.DICT_5X5_1000

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # DetectorParameters (compatibilidad con OpenCV de Ubuntu)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None


        # Bridge
        self.bridge = CvBridge()

        # Intrínsecos (llenados al recibir CameraInfo)
        self.K = None
        self.D = None

        # Suscriptores
        self.create_subscription(Image, img_topic, self.image_cb, 10)
        self.create_subscription(CameraInfo, caminfo_topic, self.camera_info_cb, 10)

        # Publicadores
        self.pose_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.anno_pub = self.create_publisher(Image, 'aruco/image_annotated', 10)

        self.get_logger().info(f'Escuchando imagen en: {img_topic}')
        self.get_logger().info(f'Escuchando camera_info en: {caminfo_topic}')
        self.get_logger().info(f'Diccionario ArUco: {dict_name}')
        self.get_logger().info(f'Tamaño de marcador: {self.marker_size} m')

    def camera_info_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float32)

    def image_cb(self, msg: Image):
        # Convertir a OpenCV (BGR8)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            # Detectar marcadores
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        annotated = frame.copy()
        poses_msg = PoseArray()
        poses_msg.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id or 'camera_color_optical_frame')

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            if self.K is not None and self.D is not None and self.marker_size > 0:
                # Definir las 4 esquinas 3D del marcador en el mundo real
                half = self.marker_size / 2.0
                obj_points = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0]
                ], dtype=np.float32)

                for i in range(len(ids)):
                    if ids[i][0] != 5:
                        continue
                    # Nuevo cálculo de Pose usando solvePnP en lugar de estimatePoseSingleMarkers
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        # Dibuja ejes (longitud ~ mitad del marcador)
                        cv2.drawFrameAxes(annotated, self.K, self.D, rvec, tvec, self.marker_size * 0.5)

                        # Pose ROS
                        pose = Pose()
                        pose.position.x = float(tvec[0][0])
                        pose.position.y = float(tvec[1][0])
                        pose.position.z = float(tvec[2][0])

                        qx, qy, qz, qw = self.rodrigues_to_quaternion(rvec)
                        pose.orientation.x = qx
                        pose.orientation.y = qy
                        pose.orientation.z = qz
                        pose.orientation.w = qw
                        poses_msg.poses.append(pose)

                        # Log distancia
                        dist = float(np.linalg.norm(tvec))
                        self.get_logger().info(
                            f'ID: {ids[i][0]} | Dist: {dist:.2f} m | XYZ=[{tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}]'
                        )
            else:
                self.get_logger().warn('Sin intrínsecos o marker_size inválido: no se estima pose.')

        # Publicar PoseArray (si hay)
        if poses_msg.poses:
            self.pose_pub.publish(poses_msg)

        # Publicar imagen anotada
        self.anno_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

    @staticmethod
    def rodrigues_to_quaternion(rvec):
        R, _ = cv2.Rodrigues(rvec.astype(np.float64))
        # Conversión a cuaternión (w last)
        tr = R[0,0] + R[1,1] + R[2,2]
        qw = np.sqrt(max(0.0, 1.0 + tr)) / 2.0
        qx = (R[2,1] - R[1,2]) / (4.0 * qw + 1e-12)
        qy = (R[0,2] - R[2,0]) / (4.0 * qw + 1e-12)
        qz = (R[1,0] - R[0,1]) / (4.0 * qw + 1e-12)
        return float(qx), float(qy), float(qz), float(qw)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()