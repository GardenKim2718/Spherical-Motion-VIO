import numpy as np
import time

class ComplementaryFilter:
    def __init__(self, alpha=0.18):
        self.alpha = alpha  # 필터의 상수
        self.angle = np.zeros(3)  # roll, pitch, yaw 초기화

    def update(self, camera_measurement, imu_measurement):
        # Increment the angle by the IMU change, then apply the filter
        self.angle += imu_measurement  # update with IMU difference
        self.angle = self.alpha * self.angle + (1 - self.alpha) * camera_measurement
        return self.angle
