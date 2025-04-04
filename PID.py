import cv2 as cv
from hardware import *
from collections import deque

"""
Use a systematic approach like Ziegler-Nichols Method or manual tuning:

Manual Tuning Approach
    
Set Ki and Kd to Zero. Start with only Kp and gradually increase it until the robot follows the line but oscillates.
Increase Kd. Add small values of Kd to dampen oscillations and smooth movement.
Add Ki. If the robot has slow corrections or drifts over time, increase Ki slightly to improve steady-state accuracy.
"""

KP = 0.175  # Proportional gain
KI = 0.0  # Integral gain
KD = 0.0  # Derivative gain
DEQUE_SIZE = 500

class PID:
    def __init__(self):
        self.previous_error = 0
        self.integral = 0
        self.error = 0
        # derivative does not need to be a class variable

        self.control_signal_history = deque(maxlen=DEQUE_SIZE)
        self.error_history = deque(maxlen=DEQUE_SIZE)

    def calculate_control_signal(self, offset):
        # PID calculations
        self.error = offset
        self.integral += self.error
        derivative = self.error - self.previous_error
        
        # Calculate control signal
        self.control_signal = (KP * self.error) + (KI * self.integral) + (KD * derivative)

        # Update previous error
        self.previous_error = self.error

        # Update histories
        self.control_signal_history.append(self.control_signal)
        self.error_history.append(self.control_signal)


    def get_differential_speed(self):
        # Calculate motor speeds
        if self.control_signal < 0: # turn left
            left_duty_cycle = MIN_DUTY_CYCLE
            right_duty_cycle = MIN_DUTY_CYCLE + abs(self.control_signal)
        else: # turn right or controlsignal=0
            left_duty_cycle = MIN_DUTY_CYCLE + self.control_signal
            right_duty_cycle = MIN_DUTY_CYCLE
        return left_duty_cycle, right_duty_cycle

