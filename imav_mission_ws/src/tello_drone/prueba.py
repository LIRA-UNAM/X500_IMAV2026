"""
Landing Script for Tello Drone (FUSIONADO Y CORREGIDO)

Camara del Tello frontal 5MP a 30 FPS, sin camara inferior
Plataforma de aterrizaje: Aruco ID:0, 80x80cm (escala local 25cm), movimiento horizontal a 0.1m/s, y con un giro 15° en Pitch.
"""
from djitellopy import Tello
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
    FF_PLATAFORMA = 0 

    TIEMPO_MAX_PERDIDA = 4.0 
    tiempo_ultima_vista = time.time() 

    cv2.namedWindow("Aterrizaje - Quique", cv2.WINDOW_NORMAL)
    frame_reader = tello.get_frame_read()

    while True:
        frame = frame_reader.frame

        # 1. Si el frame es válido, hacemos toda la visión artificial
        if frame is not None and frame.size > 0:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame_resized = cv2.resize(frame_bgr, (960, 720))
            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and id_objetivo in ids:
                tiempo_ultima_vista = time.time() 
                
                idx = np.where(ids == id_objetivo)[0][0]
                esquinas_marcador = corners[idx]

                cv2.polylines(frame_resized, [esquinas_marcador.astype(np.int32)], True, (0, 255, 0), 2)

                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [esquinas_marcador], CAMERA_MATRIX, DIST_COEFFS
                )
                tvec = tvecs[0][0]
                distancia = tvec[2]

                cx = int(np.mean(esquinas_marcador[0][:, 0]))
                cy = int(np.mean(esquinas_marcador[0][:, 1]))
                cv2.circle(frame_resized, (cx, cy), 5, (0, 0, 255), -1)

                error_x = cx - 480
                error_y = cy - 360

                lr = int(np.clip(KP_X * error_x, -30, 30))
                ud = int(np.clip(KP_Y * error_y, -30, 30))
                error_z = distancia - DIST_APROX
                fb = int(np.clip(KP_Z * error_z, -25, 25)) + FF_PLATAFORMA

                tello.send_rc_control(lr, fb, ud, 0)
                print(f"dist={distancia:.2f}m err_x={error_x} err_y={error_y} -> lr={lr} fb={fb} ud={ud}")

                if distancia < DIST_ATERRIZAJE:
                    print("Plataforma alcanzada, apagando motores.")
                    tello.send_rc_control(0, 0, 0, 0)
                    tello.land()
                    break
            else:
                # Ve imagen, pero NO ve el marcador
                tello.send_rc_control(0, 0, 0, 0)
                if (time.time() - tiempo_ultima_vista) > TIEMPO_MAX_PERDIDA:
                    print("Marcador perdido por más de 4 segundos — aterrizando por seguridad.")
                    tello.land()
                    break
            
            # Mostramos la ventana SÓLO si pudimos procesar la imagen
            cv2.imshow("Aterrizaje - Quique", frame_resized)
            
        else:
            # 2. Si hay un micro-corte de red y no llega la imagen, el dron debe frenar
            tello.send_rc_control(0, 0, 0, 0)
            if (time.time() - tiempo_ultima_vista) > TIEMPO_MAX_PERDIDA:
                print("Se perdió el video por completo durante 4 segundos — aterrizando.")
                tello.land()
                break

        # 3. ESTO ES LO QUE ARREGLA LA PANTALLA: Se ejecuta SIEMPRE en cada vuelta
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
            break
 
    cv2.destroyAllWindows()

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print(f"Batería actual: {tello.get_battery()}%")
    
    if tello.get_battery() < 15:
        print("Batería muy baja para volar, por favor cambia la pila.")
    else:
        tello.streamon()
        time.sleep(2)

        print("Despegando para prueba estática...")
        tello.takeoff()
        
        # Bajar un poco para alinear la vista si tu rampa está a 80cm de altura
        tello.move_down(25) 

        try:
            aterrizar_en_plataforma(tello, id_objetivo=0)
            
        except KeyboardInterrupt:
            # Ahora tu terminal no se congelará si haces Ctrl+C
            print("\n[ALERTA] Script abortado manualmente (Ctrl+C). Aterrizando el dron...")
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
            
        except Exception as e:
            print(f"Error inesperado durante la prueba: {e}")
            tello.send_rc_control(0, 0, 0, 0)
            tello.land()
            
        finally:
            # Cierre seguro y liberación de la terminal
            tello.streamoff()
            cv2.destroyAllWindows()
            tello.end()
            print("Conexión cerrada y recursos liberados.")