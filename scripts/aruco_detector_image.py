python3 -c "
import cv2
import numpy as np

cap = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264', cv2.CAP_FFMPEG)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) #DICT_6X6_50
params = cv2.aruco.DetectorParameters_create()

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, corner in enumerate(corners):
            pts = corner[0]
            cx = int(pts[:,0].mean())
            cy = int(pts[:,1].mean())
            cv2.putText(frame, f'ID {ids[i][0]}', (cx, cy-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
    cv2.imshow('ArUco Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
"
