import time
import cv2
from djitellopy import Tello
import numpy as np


def main():
    # Initialize the Tello drone and connect to it
    tello = Tello()
    tello.connect()
    print(f"Battery: {tello.get_battery()}%")

    # Start the video stream
    tello.streamon()
    frame_read = tello.get_frame_read()

    # Define the lower and upper bounds for the color of the window 
    lower_color = np.array([110,50,50])
    upper_color = np.array([130,255,255])

    print("Press 'q' to quit.")

    while True:
        # Get the current frame from the video stream
        frame = frame_read.frame
        if frame is None or frame.size == 0:
            continue

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the specified color range
        white_mask = cv2.inRange(hsv, lower_color, upper_color)
        result = cv2.bitwise_and(frame, frame, mask=white_mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If contours are found, draw them on the original frame
        if contours:
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 300]  # Filter small contours
            if valid_contours:
                # Find the largest contour
                largest_contour = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
                    cv2.putText(frame, "Window", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the original frame and the result
        cv2.imshow("Original Frame", frame)
        cv2.imshow("White Mask", white_mask)
        cv2.imshow("Result", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    tello.streamoff()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
