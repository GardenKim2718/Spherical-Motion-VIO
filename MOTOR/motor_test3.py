import Jetson.GPIO as GPIO
import time
import threading

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 1번 모터 GPIO 핀 정의
MOTOR1_IN1 = 6
MOTOR1_IN2 = 13
MOTOR1_IN3 = 19
MOTOR1_IN4 = 26    
Motor1Pins = [MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4]

# 2번 모터 GPIO 핀 정의c
MOTOR2_IN1 = 10
MOTOR2_IN2 = 9
MOTOR2_IN3 = 11
MOTOR2_IN4 = 5
Motor2Pins = [MOTOR2_IN1, MOTOR2_IN2, MOTOR2_IN3, MOTOR2_IN4]

# 모든 모터 핀을 설정
for pin in Motor1Pins + Motor2Pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# 유니폴라 모터용 스텝 시퀀스 정의
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
WaitTime = 0.01  # 스텝 사이의 대기 시간 (필요에 따라 조정)

# 각도 계산을 위한 상수 설정
DEG_PER_STEP = 0.9  # 모터의 스텝당 회전 각도 (변경 필요)

# 각 모터의 현재 위치와 각도 제한 설정
current_angle_motor1 = 0.0
current_angle_motor2 = 0.0

# 모터마다 최소 및 최대 각도 제한 설정
MOTOR1_ANGLE_LIMIT_MIN = 0.0   # 1번 모터 최소 각도 제한
MOTOR1_ANGLE_LIMIT_MAX = 90.0  # 1번 모터 최대 각도 제한
MOTOR2_ANGLE_LIMIT_MIN = 0.0   # 2번 모터 최소 각도 제한
MOTOR2_ANGLE_LIMIT_MAX = 90.0  # 2번 모터 최대 각도 제한

def step_motor(motor_pins, steps, direction):
    """
    지정된 모터를 주어진 스텝 수만큼 회전시키는 함수.
    
    :param motor_pins: 제어할 모터의 GPIO 핀 목록
    :param steps: 이동할 스텝 수
    :param direction: 1이면 시계방향, -1이면 반시계방향
    """
    step_counter = 0
    for _ in range(steps):
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

def rotate_motor_to_angle(motor_pins, current_angle, target_angle, min_limit, max_limit):
    """
    지정된 모터를 목표 절대 각도로 회전시키는 함수.
    
    :param motor_pins: 제어할 모터의 GPIO 핀 목록
    :param current_angle: 현재 모터 각도
    :param target_angle: 목표 회전 각도 (도)
    :param min_limit: 최소 각도 제한
    :param max_limit: 최대 각도 제한
    """
    # 각도 제한 체크
    if target_angle < min_limit or target_angle > max_limit:
        print(f"오류: 각도는 {min_limit}도에서 {max_limit}도 사이여야 합니다.")
        return current_angle
    
    # 목표 각도와 현재 각도 차이 계산
    angle_difference = target_angle - current_angle
    steps = int(abs(angle_difference) / DEG_PER_STEP)  # 필요한 스텝 수 계산

    # 방향 결정
    direction = -1 if angle_difference > 0 else 1

    # 모터 회전
    step_motor(motor_pins, steps, direction)

    # 현재 위치 업데이트
    return target_angle

def move_motors_to_angles(angle_motor1, angle_motor2):
    """
    두 모터를 동시에 지정된 각도로 회전시키는 함수.
    
    :param angle_motor1: 1번 모터의 목표 각도
    :param angle_motor2: 2번 모터의 목표 각도
    """
    global current_angle_motor1, current_angle_motor2
    
    # 스레드 생성
    thread1 = threading.Thread(target=lambda: rotate_motor_to_angle(Motor1Pins, current_angle_motor1, angle_motor1, MOTOR1_ANGLE_LIMIT_MIN, MOTOR1_ANGLE_LIMIT_MAX))
    thread2 = threading.Thread(target=lambda: rotate_motor_to_angle(Motor2Pins, current_angle_motor2, angle_motor2, MOTOR2_ANGLE_LIMIT_MIN, MOTOR2_ANGLE_LIMIT_MAX))

    # 스레드 시작
    thread1.start()
    thread2.start()

    # 스레드가 끝날 때까지 기다림
    thread1.join()
    thread2.join()

    # 각 모터의 현재 각도 업데이트
    current_angle_motor1 = angle_motor1
    current_angle_motor2 = angle_motor2

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
