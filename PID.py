import cv2 as cv
from hardware import *

KP = 0.8  # Proportional gain
KI = 0.01  # Integral gain
KD = 0.1  # Derivative gain

class PID:
    def __init__(self):
        self.previous_error = 0
        self.integral = 0
        self.error = 0
        # derivative does not need to be a class variable

    def get_offset(self, hsv, frame_width):
        """Takes HSV frame, updates offset"""

        # Create red mask and keep only bottom 100 pixels
        red_mask = cv.inRange(
            hsv, RED_HSV_RANGE['lower'], RED_HSV_RANGE['upper'])
        red_mask[:-100, :] = 0  # Keep only bottom 100 pixels

        # Find red pixels in the bottom region
        red_pixels = np.where(red_mask > 0)
            
        if len(red_pixels[1]) > 0:  # If red line is detected
            # Find max and min X coordinates of the red line
            max_x = np.max(red_pixels[1])
            min_x = np.min(red_pixels[1])
            red_center_x = (max_x + min_x) / 2

            # Calculate error (replaces offset)
            center_x = frame_width // 2
            self.error = (red_center_x - center_x)
            print("Offset: ", self.error)
        else:
            # Reset PID variables when line is lost
            self.integral = 0
            self.previous_error = 0
            # If no red line is detected, stop motors
            stop_motors()
            print("No red line detected")

    def calculate_control_signal(self):
        # PID calculations
        self.integral += self.error
        derivative = self.error - self.previous_error
        
        # Calculate control signal
        self.control_signal = (KP * self.error) + (KI * self.integral) + (KD * derivative)
        
        # Clamp control signal to [-1, 1]
        self.control_signal = max(min(self.control_signal, 1), -1)

        # Update previous error
        self.previous_error = self.error

    def get_differential_speed(self):
        # Calculate motor speeds
        if self.control_signal > 0:  # Need to turn right
            left_duty_cycle = MIN_DUTY_CYCLE
            right_duty_cycle = MIN_DUTY_CYCLE * (1 - abs(self.control_signal))
        elif self.control_signal < 0:  # Need to turn left
            left_duty_cycle = MIN_DUTY_CYCLE * (1 - abs(self.control_signal))
            right_duty_cycle = MIN_DUTY_CYCLE
        return left_duty_cycle, right_duty_cycle

