# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 11:50:10 2025

@author: jivdn
"""
#PYTHON
from picamera2 import Picamera2  #bibliotheken importeren
import cv2
import cv2.aruco as aruco
import time
import numpy as np
import math
import serial

#Seriele communicatie setup 
ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)  # Adjust port if needed
time.sleep(2)

#camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(2)

#aruco detectie setup
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)  #aruco marker uit de 5x5 bibliotheek
parameters = aruco.DetectorParameters_create()
camera_matrix = np.array([[400, 0, 320],
                          [0, 400, 240],
                          [0, 0, 1]], dtype=np.float32) #kalibratie
dist_coeffs = np.zeros((5,))
marker_length = 140  #groote van de marker

def get_yaw_from_rvec(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return math.degrees(yaw)

#wacht op "yes" sein van de Arduino
print("Waiting for Arduino...")
while True:
    if ser.in_waiting > 0:
        msg = ser.readline().decode().strip()
        print(f"Arduino says: {msg}")
        if msg.lower() == "yes":
            print("Arduino is ready. Starting detection...")
            break

# foto maken (5x) 
for i in range(5):
    print(f"\nFrame {i+1}/5")
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    timestamp = int(time.time())
    if ids is not None:
        print(f"[{timestamp}] Detected marker IDs: {ids.flatten()}")
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        rvec = rvecs[0]
        tvec = tvecs[0].flatten()
        yaw_deg = get_yaw_from_rvec(rvec)

        x = round(tvec[0], 2)
        y = round(tvec[1], 2)
        yaw = round(yaw_deg, 2)

        print(f"Sending to Arduino → X: {x}, Y: {y}, Yaw: {yaw}")
        ser.write(f"{x},{y},{yaw}\n".encode())

    else:
        print(f"[{timestamp}] No markers detected.")
        ser.write(b"no_marker\n")  # Send fallback if no marker found

    time.sleep(4)

print("Finished sending 5 frames.")
ser.write(b"done\n")  # 
ser.close()




