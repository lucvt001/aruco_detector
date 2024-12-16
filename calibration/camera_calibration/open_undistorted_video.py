import cv2
import numpy as np

# Load calibration parameters
with np.load('/home/lucvt001/camera_calibration/calibration_data.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

w, h = 1920, 1080
newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

# Open video file
vid = cv2.VideoCapture('/home/lucvt001/camera_calibration/calib1.mp4')

while True:
    result, frame = vid.read()

    if not result:
        break

    # Undistort the frame
    undistorted_frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

    # Display frame
    cv2.imshow('Undistorted Frame', undistorted_frame)
    cv2.waitKey(15)

vid.release()