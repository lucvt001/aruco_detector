#!/usr/bin/python3
import cv2
import numpy as np
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Path to the folder containing the calibration images
images_folder = '/home/lucvt001/camera_calibration/images'

columns = 15
rows = 10
# Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
objp = np.zeros((rows*columns, 3), np.float32)
objp[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Loop through all the images in the folder
images = glob.glob(images_folder + '/*.jpg')
for image_file in images:
    # Read the image
    img = cv2.imread(image_file)
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (columns, rows), None)
    
    # If corners are found, add object points and image points
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        
        # Draw and display the corners
        # cv2.drawChessboardCorners(img, (columns, rows), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(500)

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the camera matrix and distortion coefficients
print("Camera Matrix:")
print(mtx)
print("\nDistortion Coefficients:")
print(dist)

# Save the camera matrix and distortion coefficients to a file
np.savez('calibration_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# Close the OpenCV windows
cv2.destroyAllWindows()

# Compute reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

reprojection_error = mean_error / len(objpoints)
print("Reprojection Error: ", reprojection_error)