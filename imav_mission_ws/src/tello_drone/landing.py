"""
Landing Script for Tello Drone

Camara del Tello frontal 5MP a 30 FPS, sin camara inferior
Plataforma de desarrollo: entorno virtual Python ....
Plataforma de aterrizaje: Aruco ID:5, 80x80cm, movimiento horizontal a 0.1m/s, y con un giro 15° en Pitch.

"""
from djitellopy import Tello #

import cv2
import numpy as np
import time
import cv2.aruco as aruco 
CAMERA_MATRIX = np.array([
    [887.927782, 0.000000, 487.689452],
    [0.000000, 890.293534, 330.329207],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float64)
DIST_COEFFS = np.array([0.053930, -0.887274, -0.006077, -0.000734, 2.851333])

MARKER_LENGHT = 0.25

def aterrizar_en_plataforma(tello, id_objetivo=0):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    KP_X = 0.25
    KP_Y = 0.25
    KP_Z = 0.25

    DIST_APROX = 1.0
    DIST_ATERRIZAJE = 0.20

    FF_PLATAFORMA = 0 #10

    MAX_PERDIDA_FRAMES = 60

    frames_sin_marcador = 0 

    cv2.namedWindow("Aterrizaje - Quique", cv2.WINDOW_NORMAL)

    while True:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (960, 720))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None and id_objetivo in ids:
            frames_sin_marcador = 0
            idx = np.where(ids == id_objetivo)[0][0]
            esquinas_marcador = corners[idx]

            cv2.polylines(frame, [esquinas_marcador.astype(np.int32)], True, (0, 255, 0), 2)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [esquinas_marcador], CAMERA_MATRIX, DIST_COEFFS
            )
            tvec = tvecs[0][0]
            distancia = tvec[2]

            cx = int(np.mean(esquinas_marcador[0][:, 0]))
            cy = int(np.mean(esquinas_marcador[0][:, 1]))
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            error_x = cx - 480
            error_y = cy - 360


            lr = int(np.clip(KP_X * error_x, -30, 30))

            ud = int(np.clip(KP_Y * error_y, -30, 30))


            error_z = distancia - DIST_APROX
            fb = int(np.clip(KP_Z * error_z, -25, 25)) +FF_PLATAFORMA

            tello.send_rc_control(lr, fb, ud, 0)

            print(f"dist={distancia: .2f}m err_x={error_x} err_y={error_y} "
                  f"-> lr={lr} fb={fb} ud={ud}")

            if error_x < -40:
                print("Izquierda")
            elif error_x > 40:
                print("Derecha")
            if ud < 0:
                print("Bajar")
 
            if distancia < DIST_ATERRIZAJE:
                print("Plataforma alcanzada, apagando motores.")
                tello.land()
                break
        else:
            frames_sin_marcador += 1
            tello.send_rc_control(0, 0, 0, 0)
            if frames_sin_marcador > MAX_PERDIDA_FRAMES:
                print("Marcador perdido por mucho tiempo — aterrizando por seguridad.")
                tello.land()
                break
 
        cv2.imshow("Aterrizaje - Quique", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tello.land()
            break
 
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 1. Inicializar y conectar
    tello = Tello()
    tello.connect()
    print(f"Batería actual: {tello.get_battery()}%")
    
    if tello.get_battery() < 15:
        print("Batería muy baja para volar, por favor cambia la pila.")
    else:
        # 2. Encender cámara y esperar a que inicialice el flujo de video
        tello.streamon()
        time.sleep(2)

        # 3. Despegue
        print("Despegando para prueba estática...")
        tello.takeoff()
        
        # Opcional: Si pegaste el ArUco muy alto en la pared, descomenta la siguiente línea 
        # para que el dron suba un poco antes de empezar a buscarlo:
        tello.move_down(25) 

        try:
            # 4. Llamar a tu función de aterrizaje
            # OJO: Cambia el id_objetivo al número exacto del ArUco que imprimiste
            aterrizar_en_plataforma(tello, id_objetivo=0)
            
        except Exception as e:
            print(f"Error inesperado durante la prueba: {e}")
            tello.land() # Aterrizaje de emergencia si tu código falla
            
        finally:
            # Siempre apagar el flujo de video al terminar
            tello.streamoff()
            tello.end()