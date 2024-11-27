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
WaitTime = 0.016  


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
    current_absolute_angle = starting_angle  # 절대 각도를 시작 각도로 설정

    for _ in range(steps):
        # 절대 각도 업데이트
        current_absolute_angle += direction * DEG_PER_STEP
        send_angle_to_server(motor_name, current_absolute_angle)  # 서버에 각도 전송

        for pin in range(4):
            xpin = motor_pins[pin]
            GPIO.output(xpin, Seq[step_counter][pin] != 0)
        
        step_counter += direction

        # step_counter가 시퀀스의 끝을 넘지 않도록 조정
        if step_counter >= StepCount:
            step_counter = 0
        if step_counter < 0:
            step_counter = StepCount - 1

        time.sleep(WaitTime)

    	
    return current_absolute_angle  # 최종 절대 각도 반환

def rotate_motor_to_angle(motor_pins, current_angle, target_angle, min_limit, max_limit, motor_name):
    """ 지정된 모터를 목표 절대 각도로 회전시키는 함수 """
    # 각도 제한 체크
    if target_angle < min_limit or target_angle > max_limit:
        print(f"오류: {motor_name} 각도는 {min_limit}도에서 {max_limit}도 사이여야 합니다.")
        return current_angle
    
    # 목표 각도와 현재 각도 차이 계산
    angle_difference = int(target_angle/ DEG_PER_STEP) - int(current_angle/ DEG_PER_STEP)
    steps = abs(int(angle_difference))  # 필요한 스텝 수 계산

    # 방향 결정
    direction = 1 if angle_difference > 0 else -1

    # 모터 회전 (최종 절대 각도를 반환받음)
    new_angle = step_motor(motor_pins, steps, direction, current_angle, motor_name)

    # 현재 위치 업데이트
    return new_angle

def move_motors_to_angles(angle_motor1, angle_motor2):
    """ 두 모터를 동시에 지정된 각도로 회전시키는 함수 """
    global current_angle_motor1, current_angle_motor2
    
    # 스레드 생성
    thread1 = threading.Thread(target=lambda: update_motor1_angle(angle_motor1))
    thread2 = threading.Thread(target=lambda: update_motor2_angle(angle_motor2))

    # 스레드 시작
    thread1.start()
    thread2.start()

    # 스레드가 끝날 때까지 기다림
    thread1.join()
    thread2.join()
    for pin in Motor1Pins + Motor2Pins:
    	GPIO.output(pin, False)
    


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
        while True:
            user_input = input("1번 모터와 2번 모터의 목표 각도를 입력하세요 (예: 90 10, 종료하려면 'exit'): ")
            if user_input.lower() == "exit":
                break

            # 입력 값 파싱
            try:
                angle_motor1, angle_motor2 = map(float, user_input.split())
                move_motors_to_angles(angle_motor1, angle_motor2)
            except ValueError:
                print("올바른 형식으로 각도를 입력하세요. 예: 90 10")
                
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
