import Jetson.GPIO as GPIO
import time

# GPIO 설정
GPIO.setmode(GPIO.BCM)

# 1번 모터 GPIO 핀 정의
MOTOR1_IN1 = 6
MOTOR1_IN2 = 13
MOTOR1_IN3 = 19
MOTOR1_IN4 = 26    
Motor1Pins = [MOTOR1_IN1, MOTOR1_IN2, MOTOR1_IN3, MOTOR1_IN4]

# 2번 모터 GPIO 핀 정의
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
            if Seq[step_counter][pin] != 0:
                GPIO.output(xpin, True)
            else:
                GPIO.output(xpin, False)
        
        step_counter += direction

        # step_counter가 시퀀스의 끝을 넘지 않도록 조정
        if step_counter >= StepCount:
            step_counter = 0
        if step_counter < 0:
            step_counter = StepCount - 1

        time.sleep(WaitTime)

def rotate_motor_to_angle(motor, target_angle):
    """
    지정된 모터를 목표 절대 각도로 회전시키는 함수.
    
    :param motor: 1 또는 2로 모터를 지정
    :param target_angle: 목표 회전 각도 (도)
    """
    global current_angle_motor1, current_angle_motor2
    
    # 모터 선택 및 각도 제한 설정
    if motor == 1:
        current_angle = current_angle_motor1
        motor_pins = Motor1Pins
        min_limit = MOTOR1_ANGLE_LIMIT_MIN
        max_limit = MOTOR1_ANGLE_LIMIT_MAX
    elif motor == 2:
        current_angle = current_angle_motor2
        motor_pins = Motor2Pins
        min_limit = MOTOR2_ANGLE_LIMIT_MIN
        max_limit = MOTOR2_ANGLE_LIMIT_MAX
    else:
        print("잘못된 모터 선택")
        return

    # 각도 제한 체크
    if target_angle < min_limit or target_angle > max_limit:
        print(f"오류: 모터 {motor}의 각도는 {min_limit}도에서 {max_limit}도 사이여야 합니다.")
        return
    
    # 목표 각도와 현재 각도 차이 계산
    angle_difference = target_angle - current_angle
    steps = int(abs(angle_difference) / DEG_PER_STEP)  # 필요한 스텝 수 계산

    # 방향 결정
    if angle_difference > 0:
        direction = -1  # 시계방향
    elif angle_difference < 0:
        direction = 1  # 반시계방향
    else:
        return  # 이미 목표 각도에 있음

    # 모터 회전
    step_motor(motor_pins, steps, direction)

    # 현재 위치 업데이트
    if motor == 1:
        current_angle_motor1 = target_angle
    elif motor == 2:
        current_angle_motor2 = target_angle

if __name__ == "__main__":
    try:
        while True:
            motor = int(input("제어할 모터를 선택하세요 (1 또는 2, 0을 입력하면 종료): "))
            if motor == 0:
                break
            target_angle = float(input(f"모터 {motor}의 목표 절대 각도를 입력하세요: "))
            rotate_motor_to_angle(motor, target_angle)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
