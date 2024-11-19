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

class KalmanFilter:
    def __init__(self, process_noise_cov, measurement_noise_cov):
        self.state = np.zeros(3)  # 상태 초기화
        self.covariance = np.eye(3)  # 공분산 행렬 초기화

        # 프로세스 및 측정 잡음 공분산 행렬
        self.process_noise_cov = process_noise_cov
        self.measurement_noise_cov = measurement_noise_cov

    def predict(self):
        # 예측 단계
        self.state = self.state  # 상태 전파 (고정된 상태 가정)
        self.covariance = self.process_noise_cov  # 공분산 업데이트
        
    def update(self, measurement):
        # 업데이트 단계
        kalman_gain = self.covariance / (self.covariance + self.measurement_noise_cov)
        self.state = self.state + kalman_gain * (measurement - self.state)
        self.covariance = (np.eye(3) - kalman_gain) * self.covariance