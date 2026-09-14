from djitellopy import Tello
import cv2
import time

print("Conectando al dron...")
tello = Tello()
tello.connect()
print(f"Batería: {tello.get_battery()}%")

tello.streamon()
time.sleep(2) # Darle tiempo a la red para recibir el primer frame

# Forzar la creación de la ventana
cv2.namedWindow("Test de Camara", cv2.WINDOW_NORMAL)

print("Iniciando video. Presiona 'q' en la ventana para salir.")

try:
    while True:
        frame = tello.get_frame_read().frame
        
        # Validar que el frame realmente exista antes de mostrarlo
        if frame is not None:
            cv2.imshow("Test de Camara", frame)
        else:
            print("Esperando frame de video...")
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("\nDetenido manualmente.")
finally:
    tello.streamoff()
    cv2.destroyAllWindows()
    tello.end()
    print("Conexión cerrada.")