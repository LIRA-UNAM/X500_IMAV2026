# from djitellopy import Tello
# import time


# class Navigation:
#     MIN = 20
#     MAX = 500

#     def __init__(self, tello: Tello):
#         self.tello = tello
#         self.tello.connect()
#         bateria = self.tello.get_battery()

#         print(f"Batería: {bateria}%")

#         self.x_b = 0
#         self.y_b = 0

#     def takeoff(self):
#         self.tello.takeoff()
#         time.sleep(2)

#     def land(self):
#         self.tello.land()
#         time.sleep(2)

#     def coord(self, x:int, y:int, z:int):

#         self.x = x*100
#         self.y = y*100
#         self.z = z*100

#         self.x_c = self.x_b
#         self.x_b = self.x

#         self.y_c = self.y_b
#         self.y_b = self.y

#         self.current_z = self.tello.get_height()
        
#         if self.x < self.MIN or self.x > self.MAX:
#             print(f"Coordenada X fuera de rango: {self.x}")
#             return
#         if self.y < self.MIN or self.y > self.MAX:
#             print(f"Coordenada Y fuera de rango: {self.y}")
#             return
#         if self.z < self.MIN or self.z > self.MAX:
#             print(f"Coordenada Z fuera de rango: {self.z}")
#             return

#         if self.z != self.current_z:
#             if self.z > self.current_z:
#                 self.dist_z = self.z - self.current_z
#                 self.tello.move_up(self.dist_z)

#             if self.current_z > self.z:
#                 self.dist_z = self.current_z - self.z
#                 self.tello.move_down(self.dist_z)

#         elif self.z == self.current_z:
#             pass

#         if self.x > self.x_c:
#                 self.dist_x = self.x - self.x_c
#                 self.tello.move_forward(self.dist_x)

#         elif self.x < self.x_c:
#                 self.dist_x = self.x_c - self.x
#                 self.tello.move_back(self.dist_x)

#         elif self.x == self.x_c:
#             pass

#         if self.y > self.y_c:
#             self.dist_y = self.y - self.y_c
#             self.tello.move_right(self.dist_y)

#         elif self.y < self.y_c:
#             self.dist_y = self.y_c - self.y
#             self.tello.move_left(self.dist_y)

#         elif self.y == self.y_c:
#             pass

#             self.x_b = self.x
#             self.y_b = self.y
from djitellopy import Tello
import time


class Navigation:
    MIN = 20
    MAX = 500

    def __init__(self, tello: Tello):
        self.tello = tello
        self.tello.connect()
        bateria = self.tello.get_battery()

        print(f"Batería: {bateria}%")

        self.x_b = 0
        self.y_b = 0

    def takeoff(self):
        self.tello.takeoff()
        time.sleep(2)

    def land(self):
        self.tello.land()
        time.sleep(2)

    def coord(self, x:int, y:int, z:int):

        self.tx = x*100
        self.ty = y*100
        self.tz = z*100

        self.current_z = self.tello.get_height()

        dx = self.tx - self.x_b
        dy = self.ty - self.y_b
        dz = self.tz - self.current_z

        if dz != 0:
            if self.MIN <= abs(dz) <= self.MAX:
                if dz > 0:
                    self.tello.move_up(dz)
                elif dz < 0:
                    self.tello.move_down(abs(dz))

        if dx != 0:
            if self.MIN <= abs(dx) <= self.MAX:
                if dx > 0:
                    self.tello.move_forward(dx)
                elif dx < 0:
                    self.tello.move_back(abs(dx))
                self.x_b = self.tx

        if dy != 0:
            if self.MIN <= abs(dy) <= self.MAX:
                if dy > 0:
                    self.tello.move_right(dy)
                elif dy < 0:
                    self.tello.move_left(abs(dy))
                self.y_b = self.ty

if __name__ == "__main__":
    tello = Tello()
    nav = Navigation(tello)
    nav.takeoff()
    nav.coord(1, 0, 1)
    nav.land()