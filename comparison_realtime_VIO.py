'''
MEMO from CWK
WORK IN PROGRESS
removed Kalman Filter for now
-- Will be added in further update(after tuning)
-- Applied Complementary Filter and Kalman Filter 24.11.03

things to be added: fetch Intrinsic from txt file
-- Aborted
for now, change the inputs for "PinholeCamera" class
'''

'''
Usage Guide

1. Run realtime_VIO.py
2. Run AHRS.py from different terminal
3. Kill process (Anything)

If Gstreamer Related Error Occurs
execute this in terminal:
export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libpthread.so.0
'''

import os
from comparison_realtime_VO import VisualOdometry, PinholeCamera
import cv2
import time
import math
import numpy as np
import socket
import json
import filters as ck
from math import sqrt
# from Kalman import KalmanFilter
from jetcam.csi_camera import CSICamera

def run_odom(frame, img_id, array, displacement):
    grayimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    if received_data_str is not None:

        vo.update(grayimg, img_id, array, displacement)
        
        # cur_t = vo.cur_t
        cur_R = vo.cur_R
        
        if cur_R is not None:
            cur_R = np.transpose(conv_matrix) @ np.array(cur_R) @ conv_matrix
        # else:
        #     cur_R = np.eye(3)
        
        if(img_id > 0):
            angX, angY, angZ = rot2eul(cur_R)
            
        else:
            angX, angY, angZ = 0., 0., 0.

    return angX, angY, angZ


# this function does not matches current Euler Sequence -->> wait for further update
def rot2eul(Rot):
    # Ensure -1 <= Rot[2,0] <= 1 for valid arcsin input
    Rot[2, 0] = np.clip(Rot[2, 0], -1.0, 1.0)

    # Y-X-Z intrinsic rotation extraction
    angY = np.arcsin(-Rot[2, 0])            # Pitch (Y-axis)
    angZ = np.arctan2(Rot[1, 0], Rot[0, 0]) # Yaw (Z-axis)
    angX = np.arctan2(Rot[2, 1], Rot[2, 2]) # Roll (X-axis)
    
    return np.array([angX, angY, angZ])

def euler_to_rotation_matrix(x, y, z):
    # Euler Sequence: y-x-z (intrinsical rot)
    Ry = np.array([
        [np.cos(y), 0, np.sin(y)],
        [0,         1, 0],
        [-np.sin(y), 0, np.cos(y)]
    ])

    Rx = np.array([
        [1, 0,            0      ],
        [0, np.cos(x), -np.sin(x)],
        [0, np.sin(x),  np.cos(x)]
    ])

    Rz = np.array([
        [np.cos(z), -np.sin(z), 0],
        [np.sin(z),  np.cos(z), 0],
        [0,          0,           1]
    ])

    # Combined rotation matrix for Y-X-Z order: R = Rz * Rx * Ry
    R = Rz @ (Rx @ Ry)
    return R

def angle_between_euler(x1, y1, z1, x2, y2, z2):
    # Convert both Euler angle sets to rotation matrices
    R1 = euler_to_rotation_matrix(x1, y1, z1)
    R2 = euler_to_rotation_matrix(x2, y2, z2)
    
    # Compute the relative rotation matrix
    R_rel = R1.T @ R2
    
    # Calculate the angle from the relative rotation matrix
    angle = np.arccos((np.trace(R_rel) - 1) / 2)
    return angle


# Visual Odometry setup
camera = CSICamera(capture_width=960, capture_height=540, downsample = 1, capture_fps=30)
cam = PinholeCamera(width=960.0, height=540.0, fx=795.419556812739, fy=795.4373502799797, cx=598.78422838679, cy=395.0568737445681
                    , k1=-0.36912354829388827, k2=0.21550503499604207, p1=-0.001099465317271592
                    , p2=5.7741740009726946e-05, k3=-0.1015828459100714)
vo = VisualOdometry(cam)

process_noise_cov = 0.05 * np.eye(3)  # 3x3 공분산 행렬
measurement_noise_cov = 0.1 * np.eye(3)  # 3x3 공분산 행렬 (또는 단일 값으로 사용 가능)
comp_filter = ck.ComplementaryFilter()
kalman_filter = ck.KalmanFilter(process_noise_cov, measurement_noise_cov)

# Rotation matrix for AHRS-OpenCV coordin.
conv_matrix = np.array([[-1,0,0],
                        [0,-1,0],
                        [0,0,1]])

# For headless mode
os.environ["SDL_VIDEODRIVER"] = "dummy"

# kf = KalmanFilter(0.2, 0.01, 0.3)                                      #integral threshold for saturation prevention

# odometry initialization
img_id = 0

with open("comparison_odometry.txt", "w+") as f:                                           #txt file for recording odometry
    f.truncate(0)
f = open("comparison_odometry.txt", 'w')

#initializing values
rotation_radius = 0.055

execution = True
previous_err = 0.
integral = 0.
x_prev = 0.
y_prev = 0.
z_prev = 0.
displacement = 0.
p_prev = [0., 0., 0.]
r_prev = [0., 0., 0.]

try:
    now = time.time()
    # socket communication with IMU program
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 12345))
    server_socket.listen(1)
    print("Server is listening on port 12345...")
    
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    print("Ready...")
    overall_prev = time.time()
    #for _ in range(10):
    #    data = client_socket.recv(297)
    initiationtime = overall_prev

    while execution:
        # calculate time interval
        time_interval = time.time()-now
        now = time.time()     
        
        # Receive the length of the incoming data
        data_length = int(conn.recv(10).decode('utf-8').strip())
        
        # Now receive the actual data based on the received length
        data_bytes = conn.recv(data_length)
        received_data_str = data_bytes.decode('utf-8')

        # Convert JSON string to list of floats
        data_list = json.loads(received_data_str)
        data_list = [float(value) for value in data_list]
        
        # Extract Euler angles and position vector
        angles = data_list[:3]  # First three values are Euler angles obtained from INS
        p = data_list[3:]  # Last three values are position vector components obtained from INS
        
        # os.system('clear')
        print("-----------------------------")

        # obtain Rotation matrix from INS
        ROT = conv_matrix @ euler_to_rotation_matrix(x=angles[0], y=angles[1], z=angles[2]) @ np.transpose(conv_matrix)
        
        # obtain displacement from INS & update p_prev
        displacement = sqrt((p[0]-p_prev[0])**2 + (p[1]-p_prev[1])**2 + (p[2]-p_prev[2])**2)
        p_prev = p

        # take a picture
        image = camera.read()

        # run Visual Odometry
        angX,angY,angZ = run_odom(frame=image, img_id=img_id, array=ROT, displacement=displacement)
        
        # run comp. filter
        imu_angle = np.array(angles) - np.array(r_prev)
        vo_angle = np.array([angX, angY, angZ])

        # 샘플 타임스탬프 및 측정값
        dt = 0.05
        # 상보 필터 업데이트
        estimated_angle = comp_filter.update(camera_measurement=vo_angle, imu_measurement=imu_angle)
        r_prev = angles
        
        # # 칼만 필터 예측 및 업데이트
        # kalman_filter.predict()
        # kalman_filter.update(estimated_angle)

        # 각도를 degree로 변환
        estimated_angle_degrees = estimated_angle * (180.0 / np.pi)
        # kalman_state_degrees = kalman_filter.state * (180.0 / np.pi)

        print("Estimated Angles after comp filter (degrees):\n",
              estimated_angle_degrees[0], estimated_angle_degrees[1], estimated_angle_degrees[2])
        # print("Estimated Angles (roll, pitch, yaw) after Kalman Filter (degrees):\n",
        #       kalman_state_degrees[0][0], kalman_state_degrees[1][1], kalman_state_degrees[2][2])
        img_id += 1
        
        # obtain translation vector (using the VO euler angle)
        x = rotation_radius * math.sin(angY) * math.cos(angX)
        y = rotation_radius * math.sin(angX)
        z = rotation_radius * math.cos(angY) * math.cos(angX)
        
        angX = math.degrees(angX)
        angY = math.degrees(angY)
        angZ = math.degrees(angZ)
        angles = [math.degrees(value) for value in angles]
        
        overall_now = time.time()
        runtime = (overall_now - now) * 1000
        # f.write(f"{now} {kalman_state_degrees[0][0]} {kalman_state_degrees[1][1]} {kalman_state_degrees[2][2]}\n")
        # f.write(f"{now} {angX} {angY} {angZ} {angles[0]} {angles[1]} {angles[2]}\n")
        f.write(f"{now} {angX} {angY} {angZ} {estimated_angle_degrees[0]} {estimated_angle_degrees[1]} {estimated_angle_degrees[2]} {angles[0]} {angles[1]} {angles[2]} {runtime}\n")
        
        # update translation
        x_prev = x
        y_prev = y
        z_prev = z
        
        # print out results
        # print("ROT:", ROT)
        print(f"ZYX Euler: {angX:.2f}, {angY:.2f}, {angZ:.2f}")
        # print(f"ZYX Euler: {kalman_state_degrees[0]}, {kalman_state_degrees[1]}, {kalman_state_degrees[2]}")
        print("runtime: ", runtime)
        print("time interval", overall_now-overall_prev)
        overall_prev = overall_now
        

finally:
    f.close()
    camera.release()
    print("odometry saved... program terminated")