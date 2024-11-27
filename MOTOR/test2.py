import Jetson.GPIO as GPIO
import time

# GPIO 설정
GPIO.setmode(GPIO.BCM)
IN1_M1 = 6  # Motor 1
IN2_M1 = 13
IN3_M1 = 19
IN4_M1 = 26

IN1_M2 = 16  # Motor 2
IN2_M2 = 20
IN3_M2 = 21
IN4_M2 = 12

StepPins_M1 = [IN1_M1, IN2_M1, IN3_M1, IN4_M1]
StepPins_M2 = [IN1_M2, IN2_M2, IN3_M2, IN4_M2]

for pin in StepPins_M1 + StepPins_M2:
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

# 모터의 스텝을 설정
StepsPerRevolution = 200  # 모터가 360도를 회전하려면 필요한 스텝 수 (1.8도/스텝)

def degree_to_steps(degree):
    """Convert degrees to steps based on the motor's step angle (1.8 degrees per step)."""
    return int((degree / 360) * StepsPerRevolution)

StepCounter_M1 = 0
StepCounter_M2 = 0

def step_motor(MotorPins, StepCounter, direction):
    for pin in range(4):
        xpin = MotorPins[pin]
        if Seq[StepCounter][pin] != 0:
            GPIO.output(xpin, True)
        else:
            GPIO.output(xpin, False)
    
    StepCounter += direction

    if StepCounter >= StepCount:
        StepCounter = 0
    if StepCounter < 0:
        StepCounter = StepCount - 1

    return StepCounter

def move_motors(steps_m1, steps_m2):
    """Move two motors simultaneously to their target steps."""
    global StepCounter_M1, StepCounter_M2

    max_steps = max(steps_m1, steps_m2)

    for step in range(max_steps):
        if step < steps_m1:
            StepCounter_M1 = step_motor(StepPins_M1, StepCounter_M1, 1)
        if step < steps_m2:
            StepCounter_M2 = step_motor(StepPins_M2, StepCounter_M2, 1)
        time.sleep(WaitTime)

def main():
    try:
        while True:
            # User input for target degrees
            input_str = input("Enter the target degrees for Motor 1 and Motor 2 (e.g., '90 45'): ")
            try:
                target_degrees = [int(x) for x in input_str.split()]
                if len(target_degrees) != 2:
                    print("Please enter two values separated by space.")
                    continue
                
                target_degrees_m1 = target_degrees[0]
                target_degrees_m2 = target_degrees[1]

                # Convert degrees to steps
                steps_m1 = degree_to_steps(target_degrees_m1)
                steps_m2 = degree_to_steps(target_degrees_m2)

                print(f"Moving Motor 1 by {steps_m1} steps and Motor 2 by {steps_m2} steps.")

                # Move the motors
                move_motors(steps_m1, steps_m2)

            except ValueError:
                print("Invalid input. Please enter valid integers for the degrees.")
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()

