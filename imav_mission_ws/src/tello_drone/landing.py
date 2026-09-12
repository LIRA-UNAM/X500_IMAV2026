"""
Landing Script for Tello Drone

Camara del Tello frontal 5MP a 30 FPS, sin camara inferior
Plataforma de desarrollo: entorno virtual Python ....
Plataforma de aterrizaje: Aruco ID:5, 80x80cm, movimiento horizontal a 0.1m/s, y con un giro 15° en Pitch.

"""

import cv2
import numpy as np
import time
import cv2.aruco as aruco 
CAMERA_MATRIX = np.array([
    921.170702, 0.0, 459.904354],
    [0.0, 919.018377, 351.238301],
    [0.0, 0.0, 1.0]
], dtype=np.float64)
DIST_COEFFS = np.array|([-0.033458, 0.105152, 0.001256, -0.006647, 0.0])

MARKER_LENGHT = 0.08

def aterrizar_en_plataforma(tello, id_objetivo=0):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    KP_X = 0.30
    KP_Y = 0.30
    KP_Z = 0.30

    DIST_APROX = 1.0
    DIST_ATERRIZAJE = 0.35 

    FF_PLATAFORMA = 10

    MAX_PERDIDA_FRAMES = 60

    frames_sin_marcador = 0 

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


            error_z = distancia - DIST_APPROX
            fb = int(np.clip(KP_Z * error_z, -25, 25)) +FF_PLATAFORMA

            tello.send_rc_control(lr, fb, ud, 0)

            print(f"dist={distancia: .2f}m err_x={error_x} err_y={error_y} "
                  f"-> lr={lr} fb={fb} ud={ud}")

            

