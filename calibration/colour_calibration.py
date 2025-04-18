import cv2 as cv
import numpy as np
import platform


def colour_calibration():
    # Initialize webcam
    if platform.system() == 'Windows':
        cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    else:
        cap = cv.VideoCapture(0)

    # Get native resolution and swap width/height for portrait orientation
    native_width = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))  # Swapped
    native_height = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))  # Swapped

    # Set video parameters
    # Original height becomes width
    cap.set(cv.CAP_PROP_FRAME_WIDTH, native_height)
    # Original width becomes height
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, native_width)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Rotate frame 90 degrees counterclockwise
        frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)

        # Get frame dimensions
        height, width = frame.shape[:2]

        # Calculate center point
        center_x = width // 2
        center_y = height // 2

        # Draw crosshair
        cv.line(frame, (center_x - 20, center_y),
                (center_x + 20, center_y), (0, 255, 0), 2)
        cv.line(frame, (center_x, center_y - 20),
                (center_x, center_y + 20), (0, 255, 0), 2)

        # Convert frame to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Get HSV value at crosshair for different colors based on key press
        key = cv.waitKey(2) & 0xFF
        if key in [ord('r'), ord('g'), ord('b')]:
            # Get a 5x5 region around the center point
            region = hsv[center_y-2:center_y+2, center_x-2:center_x+2]

            # Calculate min and max values for each channel
            h_min, h_max = np.min(region[:, :, 0]), np.max(region[:, :, 0])
            s_min, s_max = np.min(region[:, :, 1]), np.max(region[:, :, 1])
            v_min, v_max = np.min(region[:, :, 2]), np.max(region[:, :, 2])

            color_name = {
                ord('r'): 'RED',
                ord('g'): 'GREEN',
                ord('b'): 'BLUE'
            }[key]

            print(f"\nHSV Ranges for {color_name} at crosshair:")
            print(f"\nFor use in code ({color_name}):")
            print(f"'lower': np.array([{h_min}, {s_min}, {v_min}]),")
            print(f"'upper': np.array([{h_max}, {s_max}, {v_max}])")
        elif key == ord('q'):
            break

        cv.imshow('Colour Calibration', frame)

    cap.release()
    cv.destroyAllWindows()


colour_calibration()
