import cv2 as cv
import numpy as np
import serial
import time
import math


def get_position(frame):

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array([90, 50, 20]), np.array([140, 255, 255]))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)  # quita ruido
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)  # rellena huecos

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    c = max(contours, key=cv.contourArea)
    if cv.contourArea(c) < 500:  # ignora manchas pequeñas
        return None, mask

    x, y, w, h = cv.boundingRect(c)

    cx = x + (w // 2)
    cy = y + (h // 2)

    return ((cx, cy), mask)


capture = cv.VideoCapture(0)

# configuration of the serial port

ser = serial.Serial()
ser.baudrate = 9600
ser.port = "/dev/rfcomm0"
ser.timeout = 0.05
ser.open()
time.sleep(1)

width = int(capture.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(capture.get(cv.CAP_PROP_FRAME_HEIGHT))
half_w = width / 2
half_h = height / 2
# desire position

desire_x = width // 2
desire_y = height // 2 + height // 4
while True:
    ok, frame = capture.read()

    if not ok:
        break
    (pos, mask) = get_position(frame)
    if pos is None:
        print("is None")
        ser.write(
            b"S#"
        )  # Stop message so the robot nows how to stop when is out of bounds

    else:
        cx, cy = pos

        # we pass to the robot the angle and the distance
        # between the img position and the correct one
        # we get the coodenates normalize between -1.0  and  1.0
        norm_x = (cx - half_w) / half_w
        norm_y = (cy - half_h) / half_h

        # normalize the distance (we divide with the max value)
        distance = norm_y - 0.5
        distance = max(-1.0, min(1.0, distance))
        # normalize the angle
        angle_rad = (math.atan2(-norm_y, norm_x) - (math.pi / 2)) / (math.pi / 2)

        distance = int(distance * 1000)
        angle_rad = int(angle_rad * 1000)

        msg = f"D{distance:+05d},A{angle_rad:+05d}#"
        print(msg)
        print("\n")
        ser.write(msg.encode("utf-8"))

    cv.imshow("Camara", mask)

    if cv.waitKey(1) & 0xFF == ord("q"):
        break


capture.release()
cv.destroyAllWindows()
