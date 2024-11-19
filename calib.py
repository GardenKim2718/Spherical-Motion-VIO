import numpy as np
import cv2
import sys
import os
# jetcam 모듈이 있는 경로를 sys.path에 추가
sys.path.append('/home/ircvlab/VO')

from jetcam.csi_camera import CSICamera
import ipywidgets
from IPython.display import display
from jetcam.utils import bgr8_to_jpeg

# Setup CSI Camera
camera = CSICamera(capture_width=1280, capture_height=720, downsample=1, capture_fps=20)

# Termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (e.g., 6x9 chessboard)
checkerboard_size = (9, 6)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images.
objpoints = []  # 3d points in real-world space
imgpoints = []  # 2d points in image plane

# Capture and show the image
for i in range(100):  # Capture 100 calibration images
    image = camera.read()
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        # Refine the corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(image, checkerboard_size, corners2, ret)
        display(ipywidgets.Image(value=bgr8_to_jpeg(image)))
        print(f"Checkerboard detected for image {i+1}")
    else:
        print(f"Checkerboard NOT detected for image {i+1}")

# Release the camera
camera.cap.release()

# Perform camera calibration and print intrinsic parameters if enough points are found
if len(objpoints) > 0 and len(imgpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Extract the required parameters
    focal_length_x = mtx[0, 0]  # Focal length in x
    focal_length_y = mtx[1, 1]  # Focal length in y (assuming equal focal lengths)
    center_x = mtx[0, 2]  # Principal point x
    center_y = mtx[1, 2]  # Principal point y
    
    # Distortion coefficients: k1, k2, p1, p2, k3
    k1, k2, p1, p2, k3 = dist[0]

    # Save only the numeric values to a file in one line
    with open("intrinsics.txt", "w") as file:
        file.write(f"{focal_length_x} {focal_length_y} {center_x} {center_y} {k1} {k2} {p1} {p2} {k3}\n")

    print("\nFocal length x, Focal length y, principal points, and distortion coefficients saved to intrinsics.txt")
else:
    print("Calibration failed. Ensure that enough chessboard patterns were detected.")