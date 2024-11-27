import Jetson.GPIO as GPIO
import time
import threading
import socket

GPIO.setmode(GPIO.BCM)

MOTOR1_IN1 = 6
MOTOR1_IN2 = 13
MOTOR1_IN3 = 19
MOTOR1_IN4 = 26    
Motor1Pins = [MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4]

MOTOR2_IN1 = 10
MOTOR2_IN2 = 9
MOTOR2_IN3 = 11
MOTOR2_IN4 = 5
Motor2Pins = [MOTOR2_IN4, MOTOR2_IN3, MOTOR2_IN2, MOTOR2_IN1]

for pin in Motor1Pins + Motor2Pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

Seq = [    
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

StepCount = len(Seq)
WaitTime = 0.04  

DEG_PER_STEP = 0.9  

current_angle_motor1 = 0.0
current_angle_motor2 = 0.0

MOTOR1_ANGLE_LIMIT_MIN = -1.0   
MOTOR1_ANGLE_LIMIT_MAX = 90.0  
MOTOR2_ANGLE_LIMIT_MIN = -1.0   
MOTOR2_ANGLE_LIMIT_MAX = 90.0  

SERVER_IP = '127.0.0.1'  
SERVER_PORT = 5005  

def send_angle_to_server(motor_name, angle):
    """ 소켓을 통해 서버에 모터 각도를 전송하는 함수 """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        message = f"{motor_name} angle: {angle:.1f}°"
        sock.sendto(message.encode(), (SERVER_IP, SERVER_PORT))

def step_motor(motor_pins, steps, direction, starting_angle, motor_name):
    """ 지정된 모터를 주어진 스텝 수만큼 회전시키는 함수 """
    step_counter = 0
    current_absolute_angle = starting_angle

    for _ in range(steps):
        current_absolute_angle += direction * DEG_PER_STEP
        send_angle_to_server(motor_name, current_absolute_angle)

        for pin in range(4):
            xpin = motor_pins[pin]
            GPIO.output(xpin, Seq[step_counter][pin] != 0)
        
        step_counter += direction

        if step_counter >= StepCount:
            step_counter = 0
        if step_counter < 0:
            step_counter = StepCount - 1

        time.sleep(WaitTime)

    return current_absolute_angle

def rotate_motor_to_angle(motor_pins, current_angle, target_angle, min_limit, max_limit, motor_name):
    """ 지정된 모터를 목표 절대 각도로 회전시키는 함수 """
    if target_angle < min_limit or target_angle > max_limit:
        print(f"오류: {motor_name} 각도는 {min_limit}도에서 {max_limit}도 사이여야 합니다.")
        return current_angle
    
    angle_difference = int(target_angle / DEG_PER_STEP) - int(current_angle / DEG_PER_STEP)
    steps = abs(int(angle_difference))
    direction = 1 if angle_difference > 0 else -1

    new_angle = step_motor(motor_pins, steps, direction, current_angle, motor_name)

    return new_angle

def move_motors_to_angles(angle_motor1, angle_motor2):
    """ 두 모터를 동시에 지정된 각도로 회전시키는 함수 """
    global current_angle_motor1, current_angle_motor2
    
    thread1 = threading.Thread(target=lambda: update_motor1_angle(angle_motor1))
    thread2 = threading.Thread(target=lambda: update_motor2_angle(angle_motor2))

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()
    # for pin in Motor1Pins + Motor2Pins:
    #    GPIO.output(pin, False)

def update_motor1_angle(target_angle):
    """ 1번 모터를 목표 각도로 이동하고, 현재 각도를 업데이트하는 함수 """
    global current_angle_motor1
    current_angle_motor1 = rotate_motor_to_angle(
        Motor1Pins, current_angle_motor1, target_angle,
        MOTOR1_ANGLE_LIMIT_MIN, MOTOR1_ANGLE_LIMIT_MAX, "Motor 1"
    )

def update_motor2_angle(target_angle):
    """ 2번 모터를 목표 각도로 이동하고, 현재 각도를 업데이트하는 함수 """
    global current_angle_motor2
    current_angle_motor2 = rotate_motor_to_angle(
        Motor2Pins, current_angle_motor2, target_angle,
        MOTOR2_ANGLE_LIMIT_MIN, MOTOR2_ANGLE_LIMIT_MAX, "Motor 2"
    )

if __name__ == "__main__":
    try:
        # 미리 설정된 목표 각도 시나리오
        scenarios = [
            (0, 0, 1),
            (0, 0, 1),
            (30, 0, 1),    
            (10, 20, 1),
            (10, 20, 1),
            (20, 20, 1),
            (30, 20, 1),
            (50, 40, 1),
            (30, 20, 1),
            (0, 20, 1),
            (0, 0, 1)   # 종료 시점
        ]
        
        for angle_motor1, angle_motor2, delay in scenarios:
            move_motors_to_angles(angle_motor1, angle_motor2)
            time.sleep(delay)  # 각 입력 사이의 지연 시간

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
