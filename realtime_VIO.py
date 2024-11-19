'''
MEMO from CWK
WORK IN PROGRESS
removed Kalman Filter for now
-- Will be added in further update(after tuning)

things to be added: fetch Intrinsic from txt file
for now, change the inputs for "PinholeCamera" class
'''

'''
Usage Guide

1. Run realtime_VIO.py
2. Run AHRS.py from different terminal

If Gstreamer Related Error Occurs
execute this in terminal:
export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libpthread.so.0
'''

import os
from realtime_VO_org import VisualOdometry, PinholeCamera
import cv2
import time
import math
import numpy as np
import socket
import json
import filters as ck
from math import sqrt
from jetcam.csi_camera import CSICamera

def run_odom(frame, img_id, array):
    grayimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    acc = np.zeros((3,1))
    if received_data_str is not None:

        vo.update(grayimg, img_id, array)
        
        # cur_t = vo.cur_t
        cur_R = vo.cur_R
        # VO_flag = vo.flag
        
        if cur_R is not None:
            cur_R = np.transpose(conv_matrix) @ np.array(cur_R) @ conv_matrix
        # else:
        #     cur_R = np.eye(3)
        
        if(img_id > 0):
            
            # x, y, z = cur_t[0], cur_t[1], cur_t[2]
            # kf.predict(acc)
            # kf.update(cur_t.reshape(3,1))
            # x,y,z = kf.get_state()
            angX, angY, angZ = rot2eul(cur_R)
            
        else:
            angX, angY, angZ = 0., 0., 0.

    # return angX, angY, angZ, VO_flag
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

comp_filter = ck.ComplementaryFilter(alpha=0.78)
process_noise_cov = 0.05 * np.eye(3)  # 3x3 공분산 행렬
measurement_noise_cov = 0.1 * np.eye(3)  # 3x3 공분산 행렬 (또는 단일 값으로 사용 가능)
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

with open("/home/ircvlab/VIO/odometry.txt", "w+") as f:                                           #txt file for recording odometry
    f.truncate(0)
f = open("/home/ircvlab/VIO/odometry.txt", 'w')

#initializing values
rotation_radius = 0.055

execution = True
previous_err = 0.
integral = 0.
x_prev = 0.
y_prev = 0.
z_prev = 0.
displacement = 0.
r_prev = [0., 0., 0.]
VO_flag = False

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
        angles = json.loads(received_data_str)
        angles = [float(angle) for angle in angles]
        
        # os.system('clear')
        print("-----------------------------")
        # print("Received angles:", angles)
        
        # angles = [math.radians(angle) for angle in angles]
        ROT = conv_matrix @ euler_to_rotation_matrix(x=angles[0], y=angles[1], z=angles[2]) @ np.transpose(conv_matrix)

        # take a picture
        image = camera.read()

        # run Visual Odometry
        # angX,angY,angZ, VO_flag = run_odom(frame=image, img_id=img_id, array=ROT)
        angX,angY,angZ = run_odom(frame=image, img_id=img_id, array=ROT)
        
        # run comp. filter
        imu_angle = np.array(angles) - np.array(r_prev)
        vo_angle = np.array([angX, angY, angZ])

        # 샘플 타임스탬프 및 측정값
        dt = 0.05  # 시간 간격 (0.1초)
        # 상보 필터 업데이트
        estimated_angle = comp_filter.update(camera_measurement=vo_angle, imu_measurement=imu_angle)
        r_prev = angles
        
        # # 칼만 필터 예측 및 업데이트
        # kalman_filter.predict()
        # kalman_filter.update(estimated_angle)

        # 각도를 degree로 변환
        estimated_angle_degrees = estimated_angle * (180.0 / np.pi)
        # kalman_state_degrees = kalman_filter.state * (180.0 / np.pi)

        # print("Estimated Angles after comp filter (degrees):", estimated_angle_degrees[0], estimated_angle_degrees[1], estimated_angle_degrees[2])
        # print("Estimated Angles (roll, pitch, yaw) after Kalman Filter (degrees):", kalman_state_degrees[0][0], kalman_state_degrees[1][1], kalman_state_degrees[2][2])
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
        
        # vio, imu 각각 값
        # f.write(f"{now} {kalman_state_degrees[0][0]} {kalman_state_degrees[1][1]} {kalman_state_degrees[2][2]} {angles[0]} {angles[1]} {angles[2]}\n")
        
        # vo, imu 각각 값
        # f.write(f"{now} {angX} {angY} {angZ} {kalman_state_degrees[0][0]} {kalman_state_degrees[1][1]} {kalman_state_degrees[2][2]} {angles[0]} {angles[1]} {angles[2]} {VO_flag}\n")
        f.write(f"{now} {angX} {angY} {angZ} {estimated_angle_degrees[0]} {estimated_angle_degrees[1]} {estimated_angle_degrees[2]} {angles[0]} {angles[1]} {angles[2]} {runtime} {VO_flag}\n")

        # update translation
        displacement = sqrt((x-x_prev)**2 + (y-y_prev)**2 + (z-z_prev)**2)
        x_prev = x
        y_prev = y
        z_prev = z
        
        # print out results
        print("connected with client")
        print("displacement = ",displacement)
        # print("ROT:", ROT)
        #print(f"ZYX Euler: {angX:.2f}, {angY:.2f}, {angZ:.2f}")
        print(f"IMU: {angles[0]}, {angles[1]}, {angles[2]}")
        print(f"VO: {angX}, {angY}, {angZ}")
        # print(f"Kalman: {kalman_state_degrees[0][0]}, {kalman_state_degrees[1][1]}, {kalman_state_degrees[2][2]}")
        print(f"Comp: {estimated_angle_degrees[0]}, {estimated_angle_degrees[1]}, {estimated_angle_degrees[2]}")
        print("runtime: ", runtime, " ms")
        print("time interval", overall_now-overall_prev)
        overall_prev = overall_now
        

finally:
    f.close()
    camera.release()
    print("odometry saved... program terminated")
