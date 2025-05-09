import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from hardware import *

MIN_DUTY_CYCLE = 0.25
MAX_TEST_SPEED = MAX_DUTY_CYCLE*0.7
NUM_TESTS = 3
STEP_SIZE = (MAX_TEST_SPEED-MIN_DUTY_CYCLE)/NUM_TESTS

def adjust_speed_right(speed):
    multiplier = 0.14*speed + 0.937
    speed *= multiplier
    return speed

if __name__ == "__main__":
    duty_cycles = [MIN_DUTY_CYCLE + i * STEP_SIZE for i in range(NUM_TESTS+1)]
    for dc in duty_cycles:
        print(dc)
        drive_motors(dc, adjust_speed_right(dc))
        while (right_motor_distance()<1):
            print(right_motor_distance())
        stop_motors()
        sleep(1)
        drive_motors((-1*dc), adjust_speed_right(-1*dc))
        while (right_motor_distance()>0):
           print(right_motor_distance())
        drive_motors(0, 0)
