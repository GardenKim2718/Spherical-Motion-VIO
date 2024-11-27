import Jetson.GPIO as GPIO
import time
import curses

# GPIO 설정
GPIO.setmode(GPIO.BCM)
IN1 = 6
IN2 = 13
IN3 = 19
IN4 = 26
StepPins = [IN1, IN2, IN3, IN4]

for pin in StepPins:
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
StepDir = 1  # 시계 방향으로 설정하려면 1, 반시계 방향으로 설정하려면 -1
WaitTime = 0.01  # 스텝 사이의 대기 시간 (필요에 따라 조정)

StepCounter = 0

def step_motor(direction):
    global StepCounter
    for pin in range(4):
        xpin = StepPins[pin]
        if Seq[StepCounter][pin] != 0:
            GPIO.output(xpin, True)
        else:
            GPIO.output(xpin, False)
    
    StepCounter += direction

    if StepCounter >= StepCount:
        StepCounter = 0
    if StepCounter < 0:
        StepCounter = StepCount - 1

def main(stdscr):
    global StepDir
    stdscr.nodelay(1)  # Set getch to non-blocking
    stdscr.clear()
    stdscr.addstr(0, 0, "Press arrow keys to control the motor. Press 'q' to exit.")
    stdscr.refresh()

    try:
        while True:
            key = stdscr.getch()

            if key == curses.KEY_RIGHT:
                StepDir = 1
                stdscr.addstr(2, 0, "Moving right (clockwise)...")
                stdscr.refresh()
            elif key == curses.KEY_LEFT:
                StepDir = -1
                stdscr.addstr(2, 0, "Moving left (counterclockwise)...")
                stdscr.refresh()
            elif key == ord('q'):
                stdscr.addstr(2, 0, "Exiting...                    ")
                stdscr.refresh()
                break
            else:
                key = None

            if key in [curses.KEY_RIGHT, curses.KEY_LEFT]:
                step_motor(StepDir)
                time.sleep(WaitTime)
            else:
                time.sleep(WaitTime)

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        stdscr.clear()
        stdscr.addstr(0, 0, "Program exited. Press any key to close.")
        stdscr.refresh()
        stdscr.getch()  # Wait for user input before exiting the curses window

if __name__ == "__main__":
    curses.wrapper(main)

