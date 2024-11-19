import smbus
import time
import math
import numpy as np

class mpu6050:
    # Constants
    GRAVITY_MS2 = 9.80665
    RAD_TO_DEG = 57.2957795131
    DEG_TO_RAD = 0.0174532925
    ACCEL_TRANSFORMATION_NUMBER = 0.00006103515
    GYRO_TRANSFORMATION_NUMBER = 0.01525878906
    DEFAULT_ACCEL_COEFF = 0.10
    DEFAULT_GYRO_COEFF = 1 - DEFAULT_ACCEL_COEFF

    DISCARDED_MEASURES = 100
    CALIBRATION_MEASURES = 2000

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256 = 0x00
    FILTER_BW_188 = 0x01
    FILTER_BW_98 = 0x02
    FILTER_BW_42 = 0x03
    FILTER_BW_20 = 0x04
    FILTER_BW_10 = 0x05
    FILTER_BW_5 = 0x06

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, address=0x68, bus=0):
        
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        self.filterAccelCoeff = self.DEFAULT_ACCEL_COEFF
        self.filterGyroCoeff = self.DEFAULT_GYRO_COEFF
        self.gyroXoffset = 0
        self.gyroYoffset = 0
        self.gyroZoffset = 0
        self.accXoffset = 0
        self.accYoffset = 0
        self.accZoffset = 0
        self.angGyroX = 0
        self.angGyroY = 0
        self.angGyroZ = 0
        self.angX = 0
        self.angY = 0
        self.angZ = 0
        self.v = np.array([[0.], [0.], [0.]])
        self.intervalStart = time.time()

    def read_i2c_word(self, register, retries=5):
        """Read two i2c registers and combine them with retry logic."""
        for attempt in range(retries):
            try:
                high = self.bus.read_byte_data(self.address, register)
                low = self.bus.read_byte_data(self.address, register + 1)
                value = (high << 8) + low
                if value >= 0x8000:
                    return -((65535 - value) + 1)
                else:
                    return value
            except OSError as e:
                if e.errno == 121:  # Remote I/O error
                    print(f"Remote I/O error. Retrying... ({attempt + 1}/{retries})")
                    time.sleep(0.01)  # Add a small delay before retrying
                else:
                    raise e
        raise OSError("Failed to communicate with the sensor after several retries.")

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050."""
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        actual_temp = (raw_temp / 340.0) + 36.53
        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer."""
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw=False):
        """Reads the range the accelerometer is set to."""
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        if raw:
            return raw_data
        else:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g=False):
        """Gets and returns the X, Y and Z values from the accelerometer."""
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)
        accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        accel_range = self.read_accel_range(True)
        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
        if g:
            return {'x': x, 'y': y, 'z': z}
        else:
            x = x * self.GRAVITY_MS2
            y = y * self.GRAVITY_MS2
            z = z * self.GRAVITY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope."""
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def set_filter_range(self, filter_range=FILTER_BW_256):
        """Sets the low-pass bandpass filter frequency."""
        EXT_SYNC_SET = self.bus.read_byte_data(self.address, self.MPU_CONFIG) & 0b00111000
        self.bus.write_byte_data(self.address, self.MPU_CONFIG, EXT_SYNC_SET | filter_range)

    def read_gyro_range(self, raw=False):
        """Reads the range the gyroscope is set to."""
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        if raw:
            return raw_data
        else:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope."""
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)
        gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        gyro_range = self.read_gyro_range(True)
        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier
        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        return [accel, gyro, temp]

    def calibrate(self):
        """Calibrates the sensor."""
        totalGyroX = 0
        totalGyroY = 0
        totalGyroZ = 0
        totalAccX  = 0
        totalAccY  = 0
        totalAccZ  = 0

        print("IMU Calibration in process...")
        for _ in range(self.DISCARDED_MEASURES):
            gyroData = self.get_gyro_data()
            AccData = self.get_accel_data()
            time.sleep(0.001)
        for i in range(self.CALIBRATION_MEASURES):
            gyroData = self.get_gyro_data()
            AccData = self.get_accel_data()
            totalGyroX += gyroData['x']
            totalGyroY += gyroData['y']
            totalGyroZ += gyroData['z']
            totalAccX  += AccData['x']
            totalAccY  += AccData['y']
            totalAccZ  += AccData['z']
            time.sleep(0.001)
        self.gyroXoffset = totalGyroX / self.CALIBRATION_MEASURES
        self.gyroYoffset = totalGyroY / self.CALIBRATION_MEASURES
        self.gyroZoffset = totalGyroZ / self.CALIBRATION_MEASURES
        self.accXoffset = totalAccX / self.CALIBRATION_MEASURES + self.GRAVITY_MS2
        self.accYoffset = totalAccY / self.CALIBRATION_MEASURES
        self.accZoffset = totalAccZ / self.CALIBRATION_MEASURES
        print("IMU Calibration complete")
        print(f"Acc Offset| X: {self.accXoffset:.3f}, Y: {self.accYoffset:.3f}, Z: {self.accZoffset:.3f}")
        print(f"Gyro Offset| X: {self.gyroXoffset:.3f}, Y: {self.gyroYoffset:.3f}, Z: {self.gyroZoffset:.3f}")
        
    def euler_to_rotation_matrix(self, x, y, z):
        
        """
        Calculate the rotation matrix from an intrinsic X-Z-Y sequence of Euler angles.

        Parameters:
        x (float): Rotation angle in radians around the X-axis.
        y (float): Rotation angle in radians around the Y-axis.
        z (float): Rotation angle in radians around the Z-axis.

        Returns:
        numpy.ndarray: A 3x3 rotation matrix.
        """
        
        # Rotation matrix around X-axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(x), -np.sin(x)],
            [0, np.sin(x), np.cos(x)]
        ])

        # Rotation matrix around Z-axis
        Rz = np.array([
            [np.cos(z), -np.sin(z), 0],
            [np.sin(z), np.cos(z), 0],
            [0, 0, 1]
        ])

        # Rotation matrix around Y-axis
        Ry = np.array([
            [np.cos(y), 0, np.sin(y)],
            [0, 1, 0],
            [-np.sin(y), 0, np.cos(y)]
        ])

        # Combine the rotations in intrinsic X-Z-Y order
        R = Ry @ Rz @ Rx

        return R

    def acc_angles(self): #for debugging
        """Updates the angles based on accelerometer and gyroscope data."""
        accelData = self.get_accel_data(g=False)
        accelX = accelData['x'] - self.accXoffset
        accelY = accelData['y'] - self.accYoffset
        accelZ = accelData['z'] - self.accZoffset
        # print(f"X Acc: {accelX:.3f}, Y Acc: {accelY:.3f}, Z Acc: {accelZ:.3f}")
        
        accelYAngle = math.atan2(accelZ, math.sqrt(accelX**2 + accelY**2)) * self.RAD_TO_DEG
        accelZAngle = math.atan2(accelY, -accelX) * self.RAD_TO_DEG
        # print(f"Y Angle: {accelYAngle:.1f}, Z Angle: {accelZAngle:.1f}")
        
        return {'x': 0.0, 'y': accelYAngle, 'z': accelZAngle}

    def update_angles(self):
        """Updates the angles based on accelerometer and gyroscope data."""
        accelData = self.get_accel_data(g=False)
        accelX = accelData['x'] - self.accXoffset
        accelY = accelData['y'] - self.accYoffset
        accelZ = accelData['z'] - self.accZoffset
        
        gyroData = self.get_gyro_data()
        gyroXRate = gyroData['x'] - self.gyroXoffset
        gyroYRate = gyroData['y'] - self.gyroYoffset
        gyroZRate = gyroData['z'] - self.gyroZoffset        

        accelYAngle = math.atan2(accelZ, math.sqrt(accelX**2 + accelY**2)) * self.RAD_TO_DEG
        accelZAngle = math.atan2(accelY, -accelX) * self.RAD_TO_DEG

        self.intervalEnd = time.time()
        interval = self.intervalEnd - self.intervalStart
        self.intervalStart = self.intervalEnd

        self.angGyroX += gyroXRate * interval
        self.angGyroY += gyroYRate * interval
        self.angGyroZ += gyroZRate * interval

        self.angX = self.angGyroX * self.DEG_TO_RAD
        self.angY = (self.filterAccelCoeff * accelYAngle + self.filterGyroCoeff * self.angGyroY) * self.DEG_TO_RAD
        self.angZ = (self.filterAccelCoeff * accelZAngle + self.filterGyroCoeff * self.angGyroZ) * self.DEG_TO_RAD

        return {'x': self.angX, 'y': self.angY, 'z': self.angZ}
    
    def update(self):
        """Inertial navigation using accelerometer and gyroscope data."""
        accelData = self.get_accel_data(g=False)
        accelX = accelData['x'] - self.accXoffset
        accelY = accelData['y'] - self.accYoffset
        accelZ = accelData['z'] - self.accZoffset
        
        gyroData = self.get_gyro_data()
        gyroXRate = gyroData['x'] - self.gyroXoffset
        gyroYRate = gyroData['y'] - self.gyroYoffset
        gyroZRate = gyroData['z'] - self.gyroZoffset        

        accelYAngle = math.atan2(accelZ, math.sqrt(accelX**2 + accelY**2)) * self.RAD_TO_DEG
        accelZAngle = math.atan2(accelY, -accelX) * self.RAD_TO_DEG

        self.intervalEnd = time.time()
        interval = self.intervalEnd - self.intervalStart
        self.intervalStart = self.intervalEnd

        self.angGyroX += gyroXRate * interval
        self.angGyroY += gyroYRate * interval
        self.angGyroZ += gyroZRate * interval

        self.angX = self.angGyroX * self.DEG_TO_RAD
        self.angY = (self.filterAccelCoeff * accelYAngle + self.filterGyroCoeff * self.angGyroY) * self.DEG_TO_RAD
        self.angZ = (self.filterAccelCoeff * accelZAngle + self.filterGyroCoeff * self.angGyroZ) * self.DEG_TO_RAD
        
        accXcomp = accelX + self.GRAVITY_MS2 * np.cos(self.angZ) * np.cos(self.angY)
        accYcomp = accelY - self.GRAVITY_MS2 * np.sin(self.angZ) * np.cos(self.angY)
        accZcomp = accelZ + self.GRAVITY_MS2 * np.sin(self.angY)
        
        R = self.euler_to_rotation_matrix(self.angX, self.angY, self.angZ)
        self.v = self.v + R @ np.array([[accXcomp],[accYcomp],[accZcomp]]) * interval
        displacement = self.v * interval

        return {'x': self.angX, 'y': self.angY, 'z': self.angZ, 'd': displacement}

if __name__ == "__main__":
    mpu = mpu6050(0x68, bus=7)
    mpu.calibrate()
    
    last_print_time = time.time()  # Track the last print time
    while True:
        # accel = mpu.get_accel_data(g=False)
        # accel['x'] = (accel['x'] - mpu.accXoffset)/mpu.GRAVITY_MS2
        # accel['y'] = (accel['y'] - mpu.accYoffset)/mpu.GRAVITY_MS2
        # accel['z'] = (accel['z'] - mpu.accZoffset)/mpu.GRAVITY_MS2
        
        data = mpu.update()
        # angles = mpu.acc_angles()
        
        current_time = time.time()
        
        # Print angles every 0.5 seconds
        if current_time - last_print_time >= 0.5:
            # Print Acceleration (for debugging)
            # print(f"X Acc: {accel['x']:.3f}, Y Acc: {accel['y']:.3f}, Z Acc: {accel['z']:.3f}")

            # Convert radians to degrees(for update_angle())
            data['x'] = data['x'] * (180 / math.pi)
            data['y'] = data['y'] * (180 / math.pi)
            data['z'] = data['z'] * (180 / math.pi)
            
            print(f"X Angle: {data['x']:.1f}, Y Angle: {data['y']:.1f}, Z Angle: {data['z']:.1f}, Distance: {data['d']:.5f}")
            last_print_time = current_time
        
        time.sleep(0.01)