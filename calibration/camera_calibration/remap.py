import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
import glob

# Load calibration parameters
with np.load('/home/lucvt001/camera_calibration/calibration_data.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

# Get list of all image files
images_folder = '/home/lucvt001/camera_calibration/images'
w, h = 1920, 1080

# Loop through each image file
images = glob.glob(images_folder + '/*.jpg')
newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

for image_file in images:
    # Read the image
    img = cv2.imread(image_file)

    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    print(dst.shape)

    # Display original and undistorted images
    w = int(1920 / 2)
    h = int(1080 / 2)
    img_downsized = cv2.resize(img, (w, h))
    dst_downsized = cv2.resize(dst, (w, h))
    combined = np.zeros((h, w*2, 3), dtype=np.uint8)
    combined[:, :w] = img_downsized
    combined[:, w:] = dst_downsized
    cv2.imshow('Original and Undistorted Images', combined)

    # Wait for the user to press Enter to move on to the next image
    while True:
        if cv2.waitKey(1) & 0xFF == ord('n'):
            break

cv2.destroyAllWindows()