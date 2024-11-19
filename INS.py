from MPU6050INS import mpu6050  # Ensure you import the class, not the module
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
    p = np.array([[0.], [0.], [0.]]) # position
    
    while True:
        sendingData = ''
        
        now = time.time()
        data = imu.update()
        
        if now - last > 0.05:
            last = now

            # Extract angles in radians
            x = data['z']  # Rotation around X-axis
            y = -data['x']   # Rotation around Z-axis
            z = -data['y'] # Rotation around Y-axis
            
            displacement = data['d'] # displacement

            p += displacement
            
            array = [x,y,z,p[0, 0], p[1, 0], p[2, 0]]
            
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