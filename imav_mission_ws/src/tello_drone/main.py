"""
Pumas - LIRA - UNAM
Script for Tello Drone Control

Planeación de la misión completa, completar la ruta.
"""

import os
import time
import cv2
from djitellopy import Tello

# Forzar backend X11/XCB para evitar el congelamiento de ventanas en Ubuntu
os.environ["QT_QPA_PLATFORM"] = "xcb"

from navigation import Navigation
from landing import aterrizar_en_plataforma

def main():
    print("[SISTEMA] Inicializando instancia del dron...")
    tello = Tello()
    nav = Navigation(tello)

    if tello.get_battery() < 15:
        print("[ERROR] Batería insuficiente (< 15%). Cambia la batería antes de volar.")
        return

    # PREPARACIÓN DE HARDWARE (Video)

    tello.streamon()
    frame_reader = tello.get_frame_read()
    time.sleep(2.0)

    try:
        # FASE 1: EVASIÓN DE OBSTÁCULOS
        print("\n[MISIÓN] FASE 1: Iniciando recorrido de navegación...")
        nav.takeoff()
        
        # Aquí debemos de poner todas las coordenadas de la ruta de navegación.
        nav.coord(0,0,1)
        # time.sleep(1)
        # nav.coord(0.9,2.7,1.2)
        # time.sleep(1)
        # nav.coord(0.9,2.7,2.1)
        # time.sleep(1)
        # nav.coord(0.9,2.7,2.1)
        # time.sleep(1)
        # nav.coord(1.9,2.7,2.1)
        # time.sleep(1)
        # nav.coord(1.9,2.7,0.3)
        # time.sleep(1)
        # nav.coord(2.9,2.7,0.3)
        # time.sleep(1)
        # nav.coord(3.9,2.7,0.3)
        # time.sleep(1)
        # nav.coord(3.9,2.7, 1)
        # time.sleep(1)
        # nav.coord(4.9,2.7,1)
        # time.sleep(1)
        # nav.coord(4.9,2.7,1.2)
        # time.sleep(1)
        # nav.coord(6,2.7,1.2)
        # time.sleep(1)
        # nav.coord(4.9,4.9,1.2)
        # time.sleep(1)
        # nav.rotation(180)
        # nav.coord(3,4.9,1.2)

        
        print("[MISIÓN] FASE 1 Completada. Aproximación final alcanzada.")

        # FASE 2: VISIÓN Y ATERRIZAJE AUTÓNOMO
        print("\n[MISIÓN] FASE 2: Buscando ArUco ID 0 y cediendo control a OpenCV...")
        
        # Pasamos la conexión activa y el flujo de video a tu algoritmo de visión
        aterrizar_en_plataforma(tello, frame_reader, id_objetivo=0)

    except KeyboardInterrupt:
        print("\n[ALERTA] Interrupción manual (Ctrl+C). Abortando misión...")
        # Usamos el paro de emergencia que programaste en tu clase
        nav.stop_e() 
        
    except Exception as e:
        print(f"\n[ERROR CRÍTICO] Excepción en vuelo: {e}")
        # Frena cualquier inercia y aterriza suavemente
        tello.send_rc_control(0, 0, 0, 0)
        nav.land()
        
    finally:
        # LIMPIEZA Y CIERRE SEGURO
        tello.streamoff()
        cv2.destroyAllWindows()
        tello.end()
        print("[SISTEMA] Misión finalizada. Conexión cerrada y terminal liberada.")

if __name__ == "__main__":
    main()