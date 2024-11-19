from MPU6050 import mpu6050  # Ensure you import the class, not the module
import time
import math
import numpy as np

with open("imu_odometry.txt", "w+") as f:                                           #txt file for recording odometry
    f.truncate(0)
f = open("imu_odometry.txt", 'w')

imu = mpu6050(0x68, bus=7)
imu.set_filter_range(0x03)
imu.calibrate()
last = time.time()

try:
    while True:
        sendingData = ''
        
        now = time.time()
        angle = imu.update_angles()
        
        if now - last > 0.05:

            # Extract angles in radians
            x = angle['z']  # Rotation around X-axis
            y = -1.2 * angle['x']   # Rotation around Z-axis
            z = -angle['y'] # Rotation around Y-axis
            
            # Euler Sequence: y-x-z (MPU6050: x-z-y)
            print(f"Euler: {math.degrees(x):.3f}, {math.degrees(y):.3f}, {math.degrees(z):.3f}")
            
            f.write(f"{now} {math.degrees(x)} {math.degrees(y)} {math.degrees(z)}\n")
            
            last = now
            
        time.sleep(0.005)
            
finally:
    f.close()
    print("IMU terminated")