import cv2
import numpy as np

class WindowDetector:
    """Detects windows in a video stream using color thresholding and contour detection."""
    
    def __init__(self, min_area=400): # Minimum area of the detected contour to be considered a window
        self.min_area = min_area
        # self.lower_color = np.array([0, 100, 100], dtype=np.uint8)  # Lower bound for red color in HSV
        # self.upper_color = np.array([12, 255, 255], dtype=np.uint8)  # Upper
        self.lower_white = np.array([0, 0, 180], dtype=np.uint8)
        self.upper_white = np.array([179, 45, 255], dtype=np.uint8)
    def detect(self, frame):
        """Detects windows in the given frame."""
        if frame is None or frame.size == 0:
            return frame, None, None, {"detected": False}
        
        h, w = frame.shape[:2] # Get the height and width of the frame
        drone_center_x = w // 2
        drone_center_y = h // 2
        
        # Convert the frame from RGB to BGR color space (OpenCV uses BGR)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV) # Convert the frame to HSV color space
        
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white) # Create a mask for the specified color range
        result = cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask) # Apply the mask to the original frame
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        data = {
                "detected": False,
                "cx": None,
                "cy": None,
                "error_x": 0,
                "error_y": 0,
                "area": 0.0,
                "center_x": drone_center_x,
                "center_y": drone_center_y
                }
        annotated_frame = frame_bgr.copy()
        cv2.drawMarker(annotated_frame, (drone_center_x, drone_center_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2) # Draw a marker at the center of the frame
        
        if contours:
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.min_area]
            if valid_contours:
                largest_contour = max(valid_contours, key=cv2.contourArea) # Find the largest contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    area = cv2.contourArea(largest_contour)
                    error_x = cX - drone_center_x
                    error_y = cY - drone_center_y

                    data.update({
                        "detected": True,
                        "cx": cX,
                        "cy": cY,
                        "error_x": error_x,
                        "error_y": error_y,
                        "area": area
                    })

                    cv2.drawContours(annotated_frame, [largest_contour], -1, (0, 255, 0), 2) # Draw the largest contour
                    cv2.circle(annotated_frame, (cX, cY), 5, (255, 0, 0), -1) # Draw a circle at the center of the contour
                    cv2.putText(annotated_frame, "Window", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) # Label the detected window

        return annotated_frame, data["cx"], data["cy"], data