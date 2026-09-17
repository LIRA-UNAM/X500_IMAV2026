"""
Tello window-crossing mission.

State machine:
    TAKEOFF   -> lift off and stabilize
    SEARCH    -> window not (yet) detected: hover / slow yaw scan to find it
    ALIGN     -> window detected: strafe left/right and up/down to center it,
                 creep forward slowly while centering
    CROSS     -> centered and close enough: blind forward burst to fly through
    DONE      -> stop / land

Uses djitellopy's `send_rc_control(left_right, forward_backward, up_down, yaw)`
for smooth proportional control instead of discrete move_* commands, since we
want continuous correction, not step commands.

"""

import time
import cv2
import numpy as np
from djitellopy import Tello

from window_detector import WindowDetector


# Simple PID controller

class PID:
    def __init__(self, kp, ki, kd, out_limit=100, integral_limit=500):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral_limit = integral_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error):
        now = time.time()
        dt = (now - self._prev_time) if self._prev_time else 0.033
        dt = max(dt, 1e-3)
        self._prev_time = now

        self._integral += error * dt
        self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return int(np.clip(out, -self.out_limit, self.out_limit))


# Mission controller
class TelloWindowMission:
    def __init__(self):
        self.tello = Tello()
        self.detector = WindowDetector(min_area=400)

        # --- Control gains (we need to tune these variables) ---
        # x error (pixels) -> left/right rc velocity
        self.pid_x = PID(kp=0.18, ki=0.0, kd=0.06, out_limit=35)
        # y error (pixels) -> up/down rc velocity
        self.pid_y = PID(kp=0.18, ki=0.0, kd=0.06, out_limit=35)

        # --- Thresholds ---
        self.center_tolerance_px = 25      # how close cx,cy must be to frame center
        self.creep_forward_speed = 12      # gentle forward speed while aligning
        self.cross_forward_speed = 45      # strong forward speed to fly through
        self.cross_duration_s = 2.2        # how long to fly forward "blind" through the window
        self.close_area_threshold = 28000  # contour area (px^2) considered "close enough" to cross
        self.align_hold_frames = 8         # consecutive well-centered frames required before crossing

        # --- Safety ---
        self.search_timeout_s = 15         # give up searching after this long
        self.lost_detection_timeout_s = 3  # abort ALIGN if detection lost this long
        self.min_battery = 15
        self.battery_check_interval_s = 3  # don't query battery every frame, it floods the command link

        self.frame_reader = None
        self._aligned_streak = 0

    def connect(self):
        self.tello.connect()
        print(f"Battery: {self.tello.get_battery()}%")
        if self.tello.get_battery() < self.min_battery:
            raise RuntimeError("Battery too low to fly safely.")
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()
        time.sleep(1.0)

    def get_frame(self):
        """Return a BGR frame ready for detect() (which expects RGB input)."""
        frame_bgr = self.frame_reader.frame
        if frame_bgr is None:
            return None
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return frame_rgb


    def run(self):
        self.connect()
        self.tello.takeoff()
        time.sleep(2)
        self.tello.send_rc_control(0, 0, 0, 0)

        try:
            state = "SEARCH"
            state_enter_time = time.time()
            last_detected_time = time.time()
            last_battery_check = time.time()

            while True:
                frame = self.get_frame()
                if frame is None:
                    continue

                annotated, cx, cy, data = self.detector.detect(frame)
                cv2.imshow("Tello - Window Mission", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Manual abort.")
                    break

                # global battery watchdog - throttled, don't hammer the command link every frame
                now = time.time()
                if now - last_battery_check > self.battery_check_interval_s:
                    last_battery_check = now
                    if self.tello.get_battery() < self.min_battery:
                        print("Battery critical, landing.")
                        break

                # STATE: SEARCH
                if state == "SEARCH":
                    self.tello.send_rc_control(0, 0, 0, 0)
                    if data["detected"]:
                        print("Window found, switching to ALIGN.")
                        self.pid_x.reset()
                        self.pid_y.reset()
                        self._aligned_streak = 0
                        state = "ALIGN"
                        state_enter_time = time.time()
                        last_detected_time = time.time()
                    else:
                        # slow yaw scan to look for the window
                        self.tello.send_rc_control(0, 0, 0, 15)
                        if time.time() - state_enter_time > self.search_timeout_s:
                            print("Search timed out, no window found. Landing.")
                            break

                # STATE: ALIGN
                elif state == "ALIGN":
                    if data["detected"]:
                        last_detected_time = time.time()
                        error_x = data["error_x"]
                        error_y = data["error_y"]

                        lr = self.pid_x.update(error_x)          # + error_x -> window right -> move right
                        # image y grows downward; if window center is above frame center (error_y<0)
                        # we want to move up, i.e. positive up_down speed
                        ud = -self.pid_y.update(error_y)

                        centered = (abs(error_x) < self.center_tolerance_px and
                                    abs(error_y) < self.center_tolerance_px)

                        fb = self.creep_forward_speed if centered else 0

                        self.tello.send_rc_control(lr, fb, ud, 0)

                        if centered:
                            self._aligned_streak += 1
                        else:
                            self._aligned_streak = 0

                        close_enough = data["area"] >= self.close_area_threshold
                        if self._aligned_streak >= self.align_hold_frames and close_enough:
                            print("Centered and close enough, switching to CROSS.")
                            state = "CROSS"
                            state_enter_time = time.time()
                    else:
                        # brief detection dropout: keep last command briefly, then abort
                        if time.time() - last_detected_time > self.lost_detection_timeout_s:
                            print("Lost the window during alignment, landing.")
                            break

                # STATE: CROSS 
                elif state == "CROSS":
                    self.tello.send_rc_control(0, self.cross_forward_speed, 0, 0)
                    if time.time() - state_enter_time > self.cross_duration_s:
                        print("Crossed the window.")
                        state = "DONE"

                # STATE: DONE 
                elif state == "DONE":
                    self.tello.send_rc_control(0, 0, 0, 0)
                    break

        finally:
            self.tello.send_rc_control(0, 0, 0, 0)
            time.sleep(1.0)

            landed = False
            for attempt in range(3):
                try:
                    self.tello.land()
                    landed = True
                    break
                except Exception as e:
                    print(f"Landing attempt {attempt + 1} failed: {e}")
                    time.sleep(1.5)

            if not landed:
                # Last resort: cuts motors immediately, drone will drop.
                print("WARNING: land failed repeatedly, sending emergency stop.")
                try:
                    self.tello.emergency()
                except Exception as e:
                    print(f"Emergency stop also failed: {e}")

            try:
                self.tello.streamoff()
            except Exception as e:
                print(f"streamoff issue: {e}")
            cv2.destroyAllWindows()


if __name__ == "__main__":
    mission = TelloWindowMission()
    mission.run()
