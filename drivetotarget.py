import cv2 as cv
import numpy as np
from math import log
from hardware import *
import platform
import PID
from collections import deque
import matplotlib.pyplot as plt

STARTUP_FRAMES = 15  # number of frames to throw away on startup
SECTION_OF_FRAME = 0.8  # bottom percentage of frame to declare as regiion of interest
VERBOSE = False
DELAY = 1
MAX_TURNS = 9

MIN_LEN = 0.1  # m
MAX_LEN = 0.5 # m
M = (MAX_DUTY_CYCLE-MIN_DUTY_CYCLE)/(MAX_LEN-MIN_LEN)
B = MAX_DUTY_CYCLE-MAX_LEN*M
REAL_M = 1
REAL_B = 0
BUFFER_CAPACITY = 5

def get_ROI(height, width):
    """
    Creates a binary mask with:
    - The bottom 20% of the frame filled white
    - A triangle above that, connecting the top center to the bottom corners of the 20% line
    - The top 40% of the frame is blacked out (not part of the ROI)

    Returns:
        region_of_interest (np.ndarray): A binary mask (uint8) with ROI marked as 255
    """
    # Create a blank (black) mask
    region_of_interest = np.zeros((height, width), dtype=np.uint8)

    # Bottom 30% starting from 60% height
    roi_start_y = int(height * (1-SECTION_OF_FRAME))  # whatever number is here, the 1-x is the bottom percent that is visible
    roi_end_y = height

    # Middle 50% in X direction
    roi_start_x = int(width * 0)
    roi_end_x = int(width * 1)

    region_of_interest[roi_start_y:roi_end_y, roi_start_x:roi_end_x] = 255

    return region_of_interest

def drive_to_target_main():
    # Initialize webcam
    if platform.system() == 'Windows':
        cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    else:
        cap = cv.VideoCapture(0)

    native_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    native_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))

    # Initialize plots
    if VERBOSE:
        plt.ion()  # Interactive mode on
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 6))
        line1, = ax1.plot([], [], 'r-', label="Error")
        line2, = ax2.plot([], [], 'b-', label="Control Signal")
        ax1.set_ylim(-1, 1)
        ax2.set_ylim(-1, 1)
        ax1.legend()
        ax2.legend()
    
    pid = PID.PID()
    target = False
    region_of_interest = get_ROI(native_height, native_width)
    buffer = deque(maxlen=BUFFER_CAPACITY)  # mid_x
    got_line = False  # bool on whether or not line is tracked, prevents premature exit of program
    count = STARTUP_FRAMES
    while not target:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)  # Convert to HSV color space
        mask1 = cv.inRange(hsv, RED_HSV_RANGE['lower_red1'], RED_HSV_RANGE['upper_red1'])  # Create masks for red color
        mask2 = cv.inRange(hsv, RED_HSV_RANGE['lower_red2'], RED_HSV_RANGE['upper_red2'])  # Create masks for red color

        mask = cv.bitwise_or(mask1, mask2)  # get all red
        mask = cv.bitwise_and(mask, region_of_interest)  # isolate for ROI
        red_regions = cv.bitwise_and(frame, frame, mask=mask)  # Apply mask to isolate red regions

        # # Convert the mask to grayscale for edge detection
        gray = cv.cvtColor(red_regions, cv.COLOR_BGR2GRAY)

        # # [*4] Apply Canny edge detection
        edges = cv.Canny(gray, 50, 150)

        # # [*5] Use HoughLinesP to detect line segments
        lines = cv.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        points = []
        mask_bgr = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        if lines is not None:
            got_line = True
            for line in lines:
                x1, y1, x2, y2 = line[0]  # Unpack line endpoints
                points.append((x1, y1))
                points.append((x2, y2))
                cv.line(mask_bgr, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Draw line bounds in red
            highest_point = min(points, key=lambda p: p[0])  # Point with smallest x
            lowest_point = max(points, key=lambda p: p[0])   # Point with largest x
            mid_x = float(lowest_point[0] + (highest_point[0]-lowest_point[0])/2)
            buffer.append(mid_x)

        else:
            print("No red line detected")
            stop_motors()

        # UPDATE MID_X AND OFFSET regardless of if we detect line or not
        if buffer:
            average_mid_x = sum(buffer) / len(buffer)
            offset = (native_width/2) - average_mid_x
            scaled_offset = -1*offset/(native_width/2)
            # Draw vertical line of offset
            cv.line(mask_bgr, (int(average_mid_x), 0), (int(average_mid_x), native_height), (255, 255, 0), 2)  
            pid.calculate_control_signal(scaled_offset)
            left_duty_cycle, right_duty_cycle = pid.get_differential_speed()
            print(f"{left_duty_cycle:.3f}, {right_duty_cycle:.3f}")

            # THROW AWAY FIRST 10 FRAMES
            if count == 0:
                drive_motors(left_duty_cycle, right_duty_cycle)
                target = is_target_close(hsv)
            else: # count is positive
                count -= 1
        elif got_line:  # buffer has been cleared after line identified at some point
            print("Red line out of sight. Stopping motors.")
            stop_motors()
            break

        # Display the original frame with detected lines
        cv.imshow('Red Line Detection', mask_bgr)
        
        # Update plots
        if VERBOSE:
            line1.set_data(range(len(pid.error_history)), list(pid.error_history))
            line2.set_data(range(len(pid.control_signal_history)), list(pid.control_signal_history))
            ax1.set_xlim(0, len(pid.error_history))
            ax2.set_xlim(0, len(pid.control_signal_history))
            plt.pause(0.001)

        # Break loop on user interrupt (e.g., 'q' key press)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    drive_to_target_main()