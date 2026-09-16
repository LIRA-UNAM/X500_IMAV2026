import time
import cv2
import cv2.aruco as aruco
import numpy as np

# ==================== CALIBRACIÓN Y PARÁMETROS ====================
CAMERA_MATRIX = np.array(
    [
        [887.927782, 0.000000, 487.689452],
        [0.000000, 890.293534, 330.329207],
        [0.000000, 0.000000, 1.000000],
    ],
    dtype=np.float64,
)
DIST_COEFFS = np.array([0.053930, -0.887274, -0.006077, -0.000734, 2.851333])

MARKER_LENGTH = 0.25  # Longitud real del lado del marcador en metros (25 cm)

def estimate_marker_pose(corners, marker_length, camera_matrix, dist_coeffs):
    """Reemplazo directo de cv2.aruco.estimatePoseSingleMarkers usando cv2.solvePnP."""
    half = marker_length / 2.0
    obj_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    img_points = corners.reshape((4, 2)).astype(np.float32)
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return rvec, tvec

def aterrizar_en_plataforma(tello, frame_reader, id_objetivo=0):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Ganancias de control proporcional
    KP_X = 0.22
    KP_Y = 0.22
    KP_Z = 0.22

    DIST_APROX = 0.0
    DIST_ATERRIZAJE = 0.10  # Distancia umbral en metros para iniciar el corte/aterrizaje
    FF_PLATAFORMA = 10

    TIEMPO_MAX_PERDIDA = 4.0
    tiempo_ultima_vista = time.time()

    cv2.namedWindow("Aterrizaje - Tello", cv2.WINDOW_NORMAL)

    while True:
        frame = frame_reader.frame

        if frame is not None and frame.size > 0:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_resized = cv2.resize(frame_bgr, (960, 720))
            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and id_objetivo in ids:
                tiempo_ultima_vista = time.time()

                idx = np.where(ids == id_objetivo)[0][0]
                esquinas_marcador = corners[idx]

                cv2.polylines(
                    frame_resized,
                    [esquinas_marcador.astype(np.int32)],
                    True,
                    (0, 255, 0),
                    2,
                )

                rvec, tvec = estimate_marker_pose(
                    esquinas_marcador[0], MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS
                )

                distancia = float(tvec[2][0])
                cx = int(np.mean(esquinas_marcador[0][:, 0]))
                cy = int(np.mean(esquinas_marcador[0][:, 1]))
                cv2.circle(frame_resized, (cx, cy), 5, (0, 0, 255), -1)

                error_x = cx - 480
                error_y = cy - 360
                error_z = distancia - DIST_APROX

                lr = int(np.clip(KP_X * error_x, -30, 30))
                ud = int(np.clip(-KP_Y * error_y, -30, 30))
                fb = int(np.clip(KP_Z * error_z * 100, -25, 25)) + FF_PLATAFORMA

                tello.send_rc_control(lr, fb, ud, 0)
                print(f"Dist: {distancia:.2f}m | ErrX: {error_x} ErrY: {error_y} -> LR: {lr} FB: {fb} UD: {ud}")

                if distancia < DIST_ATERRIZAJE:
                    print("[INFO] Plataforma alcanzada. Aterrizando...")
                    tello.send_rc_control(0, 0, 0, 0)
                    tello.emergency()
                    break
            else:
                tello.send_rc_control(0, 0, 0, 0)
                if (time.time() - tiempo_ultima_vista) > TIEMPO_MAX_PERDIDA:
                    print("[WARN] Marcador perdido por más de 4 segundos. Aterrizaje de seguridad.")
                    tello.emergency()
                    break

            cv2.imshow("Aterrizaje - Tello", frame_resized)
        else:
            tello.send_rc_control(0, 0, 0, 0)
            if (time.time() - tiempo_ultima_vista) > TIEMPO_MAX_PERDIDA:
                print("[WARN] Señal de video perdida. Aterrizando...")
                tello.land()
                break

        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[INFO] Abortado con tecla 'q'.")
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
            break

    cv2.destroyAllWindows()