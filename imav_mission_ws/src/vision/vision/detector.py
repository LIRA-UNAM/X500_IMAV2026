import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import math
from ultralytics import YOLO

class Detector(Node):
    def __init__(self):
        super().__init__('cnn_angle_detector')
        self.bridge = CvBridge()
        
        self.get_logger().info("Cargando modelo YOLOv8...")
        self.model = YOLO('yolov8n.pt') #Para pruebas utilziamos el preentrenado
        
        self.fx = self.fy = self.cx = self.cy = None
        
        # Suscripciones a la Orbbec 335L
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.img_cb, 10)
        
        self.get_logger().info("Esperando conexión con Orbbec...")

    def info_cb(self, msg: CameraInfo):
        # Guardar la matriz de intrínsecos
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def img_cb(self, msg: Image):
        if self.fx is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Utilizar YOLOv8. El parámetro classes=[62] filtra solo Monitores/TV
        results = self.model.predict(source=cv_img, classes=[62], verbose=False)
        
        for r in results:
            for box in r.boxes:
                # Obtener bounding box del monitor
                x1, y1, x2, y2 = box.xyxy[0]
                
                # Calcular el centro en píxeles (u, v)
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                
                # Convertir de Píxeles a Ángulos
                yaw_rad = math.atan((u - self.cx) / self.fx)
                pitch_rad = math.atan((v - self.cy) / self.fy)
                
                yaw_deg = math.degrees(yaw_rad)
                pitch_deg = math.degrees(pitch_rad)
                
                self.get_logger().info(
                    f"Monitor detectado -> Centro(u={u}, v={v}) | Ángulos(Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°)"
                )
                
                cv2.rectangle(cv_img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.circle(cv_img, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(cv_img, f"Yaw:{yaw_deg:.1f} Pitch:{pitch_deg:.1f}", 
                            (u - 20, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Prueba", cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()