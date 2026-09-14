import cv2
import numpy as np
import time
from djitellopy import Tello

# --- Configuración del Tablero de Ajedrez ---
# OJO: Estos valores dependen de tu tablero impreso.
# COLUMNAS y FILAS se refieren a las intersecciones internas (esquinas), no a los cuadros.
# Si tu tablero tiene 9x7 cuadros, las esquinas internas son 8x6.
COLUMNAS = 9
FILAS = 6
# Tamaño real del lado de un cuadrado en metros (ej. 25 mm = 0.025 m)
TAMANO_CUADRO = 0.02

# Criterios de terminación para la calibración subpíxel
criterios = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparar puntos 3D del mundo real
objp = np.zeros((FILAS * COLUMNAS, 3), np.float32)
objp[:, :2] = np.mgrid[0:COLUMNAS, 0:FILAS].T.reshape(-1, 2)
objp = objp * TAMANO_CUADRO

# Arrays para almacenar puntos 3D y 2D de todas las imágenes
puntos_3d = [] # Puntos en el mundo real 3D
puntos_2d = [] # Puntos 2D en el plano de la imagen

# Iniciar dron
tello = Tello()
tello.connect()
print(f"Batería Tello: {tello.get_battery()}%")
tello.streamon()

print("Presiona 'c' para capturar una imagen correcta del tablero.")
print("Presiona 'q' para terminar de capturar y calcular la calibración.")

capturas = 0

while True:
    frame = tello.get_frame_read().frame
    # Es vital usar la misma resolución que usarás en el aterrizaje (960x720)
    frame = cv2.resize(frame, (960, 720))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Buscar esquinas del tablero
    ret, esquinas = cv2.findChessboardCorners(gray, (COLUMNAS, FILAS), None)
    
    frame_display = frame.copy()
    if ret:
        # Dibujar esquinas para feedback visual
        cv2.drawChessboardCorners(frame_display, (COLUMNAS, FILAS), esquinas, ret)
        
    cv2.imshow('Calibracion de Camara Tello', frame_display)
    
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('c') and ret:
        # Refinar las esquinas a nivel subpíxel
        esquinas_refinadas = cv2.cornerSubPix(gray, esquinas, (11, 11), (-1, -1), criterios)
        puntos_3d.append(objp)
        puntos_2d.append(esquinas_refinadas)
        capturas += 1
        print(f"Captura {capturas} guardada. Cambia el ángulo o distancia del dron.")
        
    elif tecla == ord('q'):
        break

cv2.destroyAllWindows()
tello.streamoff()

if capturas < 10:
    print("Se necesitan al menos 10-15 capturas para una buena calibración. Intenta de nuevo.")
else:
    print("Calculando matriz de cámara... (Esto puede tomar unos segundos)")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(puntos_3d, puntos_2d, gray.shape[::-1], None, None)
    
    print("\n=========================================")
    print("CALIBRACIÓN EXITOSA")
    print("=========================================\n")
    print("CAMERA_MATRIX = np.array([")
    for fila in mtx:
        print(f"    [{fila[0]:.6f}, {fila[1]:.6f}, {fila[2]:.6f}],")
    print("], dtype=np.float64)\n")
    
    dist_flat = dist[0]
    print(f"DIST_COEFFS = np.array([{dist_flat[0]:.6f}, {dist_flat[1]:.6f}, {dist_flat[2]:.6f}, {dist_flat[3]:.6f}, {dist_flat[4]:.6f}])\n")
    print("Copia y pega estos valores en tu script de aterrizaje.")