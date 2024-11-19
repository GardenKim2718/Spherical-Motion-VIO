from MPU6050 import mpu6050  # Ensure you import the class, not the module
import time
import socket
import math
import numpy as np
import json

imu = mpu6050(0x68, bus=7)
imu.set_filter_range(0x03)
imu.calibrate()
last = time.time()

# 서버 소켓을 특정 포트에 바인딩
server_host = '127.0.0.1'
server_port = 12345
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_host, server_port))

try:
    while True:
        sendingData = ''
        
        now = time.time()
        angle = imu.update_angles()
        
        if now - last > 0.05:
            last = now

            # Extract angles in radians
            x = angle['z']  # Rotation around X-axis
            y = -angle['x']   # Rotation around Z-axis
            z = -angle['y'] # Rotation around Y-axis

            array = [x,y,z]
            
            # Euler Sequence: y-x-z (MPU6050: x-z-y)
            print("Euler:", math.degrees(x), math.degrees(y), math.degrees(z))
            
            data_str = json.dumps([f"{val:.6f}" for val in array])
            data_bytes = data_str.encode('utf-8')

            # Send the length of data first, followed by the actual data
            data_length = f"{len(data_bytes):<10}".encode('utf-8')
            client_socket.sendall(data_length + data_bytes)
            
            # print("Sent angles:", array)
            
            time.sleep(0.005)
            
finally:
    client_socket.close()
    print("IMU terminated")