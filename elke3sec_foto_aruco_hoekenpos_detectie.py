# -*- coding: utf-8 -*-
"""
Created on Sun Jun  8 12:04:17 2025

@author: jivdn
"""

from picamera2 import Picamera2              #Controls rasberry pi
import cv2                                   #Opencv for image proccesing
import cv2.aruco as aruco                    #Opencv module specifically for Aruco markers
import time                                  #voor timing
import numpy as np                           #voor matrixen
import math                                  #voor wiskunde





#hoek
def get_yaw_from_rvec(rvec):                                       #functie die rotatie vector converteert in een maxtix
    rotation_matrix, _ = cv2.Rodrigues(rvec)                        #conveteert rotatie vector naar matrix cv2.Rodrigue
    yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])  
    return math.degrees(yaw)                                               #converteert van radialen naar graden 

#Camera calibration 
camera_matrix = np.array([[400, 0, 320],
                          [0, 400, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,))  #

#
marker_length = 140    #14x14cm marker gebruikt

# initializeer camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(2)                                                              #wacht 2 seconden zodat de camera kan opwarmen en beeld genereren
# --- Set up ArUco detection ---
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)             #hier word de juiste dictionary gespecificeerd, voor het experiment kwam de arucomarker uit: aruco.DICT_6X6_250
parameters = aruco.DetectorParameters_create()

print("Starting detection loop. Press Ctrl+C to stop.")  #is te zien in de terminal bij het opstarten van de programma

try:
    while True:
        # maakt foto
        frame = picam2.capture_array()
        timestamp = int(time.time())
        filename = f"capture_{timestamp}.jpg"
        cv2.imwrite(filename, frame)

        # converteerd de foto naar grijswaardes
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detecteer markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            print(f"[{timestamp}] Detected marker IDs: {ids.flatten()}")
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Deze regel berekent waar de ArUco-marker is en hoe hij gedraaid is ten opzichte van de camera,
            #met behulp van de markerhoeken en camerainstellingen.
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                # teken assen
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

                # translaties (mm)
                tvec = tvecs[i].flatten()

                # rotatie (hoek)
                yaw_deg = get_yaw_from_rvec(rvecs[i])  #rvec=rotatie vector

               
                print(f" rotation (Yaw): {yaw_deg:.2f}°")           #print rotatie
                print(f"  Position (mm):")                          
                print(f"    X = {tvec[0]:.2f}")                     # x positie in mm
                print(f"    Y = {tvec[1]:.2f}")                    # y positie in mm
                print(f"    Z = {tvec[2]:.2f}")                    #z positie in mm (niet heel belangrijk) meer om te zien hoe dicht bij/hoe groot de aruco marker is maar die zal altijd op 2m zijn

        else:
            print(f"[{timestamp}] No markers detected.")   #print alleen als er geen markers gedetecteerd zijn

        # Display result
        cv2.imshow("ArUco Detection", frame)  #toon gemaakte foto in "ArUco Detection" window
        cv2.waitKey(1000)                     #wacht 1 seconden

        time.sleep(3)   #maakt foto elke 3 seconden
                        #dus elke foto word 4 seconden getoond en dan komt een nieuwe

except KeyboardInterrupt:               #cntrl + C dan stopt programma
    print("Detection stopped.")
    cv2.destroyAllWindows()             #sluit alles
