#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

# --- Configuración del Tablero de Ajedrez ---
COLUMNAS = 9
FILAS = 6
TAMANO_CUADRO = 0.02  # 20 mm

criterios = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((FILAS * COLUMNAS, 3), np.float32)
objp[:, :2] = np.mgrid[0:COLUMNAS, 0:FILAS].T.reshape(-1, 2)
objp = objp * TAMANO_CUADRO

class OrbbecCalibrator(Node):
    def __init__(self):
        super().__init__('orbbec_calibrator')
        self.bridge = CvBridge()
        
        # Suscripción al tópico de color de la Orbbec 335L
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )
        
        self.puntos_3d = []
        self.puntos_2d = []
        self.capturas = 0
        self.image_shape = None
        
        self.get_logger().info("=== CALIBRADOR ORBBEC INICIADO ===")
        self.get_logger().info("Asegúrate de que la ventana de OpenCV esté seleccionada.")
        self.get_logger().info("Presiona 'c' para capturar el tablero.")
        self.get_logger().info("Presiona 'q' para finalizar y calcular la matriz.")

    def image_cb(self, msg):
        # Convertir de ROS 2 a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Capturar la resolución real de la cámara para la fórmula matemática
        if self.image_shape is None:
            self.image_shape = frame.shape[:2]
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Buscar esquinas
        ret, esquinas = cv2.findChessboardCorners(gray, (COLUMNAS, FILAS), None)
        
        frame_display = frame.copy()
        if ret:
            cv2.drawChessboardCorners(frame_display, (COLUMNAS, FILAS), esquinas, ret)
            
        cv2.imshow('Calibracion Orbbec Gemini 335L', frame_display)
        
        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord('c') and ret:
            esquinas_refinadas = cv2.cornerSubPix(gray, esquinas, (11, 11), (-1, -1), criterios)
            self.puntos_3d.append(objp)
            self.puntos_2d.append(esquinas_refinadas)
            self.capturas += 1
            self.get_logger().info(f"Captura {self.capturas} guardada. Cambia la posición del tablero.")
            
        elif tecla == ord('q'):
            self.calibrar_y_salir()

    def calibrar_y_salir(self):
        cv2.destroyAllWindows()
        if self.capturas < 10:
            self.get_logger().error(f"Solo tomaste {self.capturas} capturas. Se necesitan al menos 10. Intenta de nuevo.")
        else:
            self.get_logger().info("Calculando matriz de la cámara... Esto tomará unos segundos.")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.puntos_3d, self.puntos_2d, self.image_shape[::-1], None, None
            )
            
            print("\n=========================================")
            print("CALIBRACIÓN EXITOSA - ORBBEC GEMINI 335L")
            print("=========================================\n")
            print("CAMERA_MATRIX = np.array([")
            for fila in mtx:
                print(f"    [{fila[0]:.6f}, {fila[1]:.6f}, {fila[2]:.6f}],")
            print("], dtype=np.float64)\n")
            
            dist_flat = dist[0]
            print(f"DIST_COEFFS = np.array([{dist_flat[0]:.6f}, {dist_flat[1]:.6f}, {dist_flat[2]:.6f}, {dist_flat[3]:.6f}, {dist_flat[4]:.6f}])\n")
            print("Reemplaza estos valores en tu nodo de ArUco o aterrizaje.")
        
        # Detener el nodo
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = OrbbecCalibrator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()