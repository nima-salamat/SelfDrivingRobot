import cv2
import numpy as np
from config import (
    HOUGH_RHO,
    HOUGH_THETA,
    HOUGH_THRESHOLD,
    HOUGH_MAX_LINE_GAP,
    HOUGH_MIN_LINE_LEN,
    BLUR_KERNEL,
    CANNY_HIGH,
    CANNY_LOW,
    MIN_SLOPE,
    HISTORY_LEN,
    THRESHOLD_MID,
    THRESHOLD_SHARP,
    THRESHOLD_SLOW,
    FIXED_CENTER,
    
    
    
    
)

class LaneDetector:
    def __init__(self, camera, ser):
        self.camera = camera
        # Initialize lane tracking variables
        self.LANE_CENTER_HISTORY = []
        self.previous_lane_center = self.camera.width / 2
        self.ser = ser
    
    def detect(self, frame):
        

        

        # Preprocess image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

        # Apply ROI
        roi = edges[
            self.camera.ROI_Y_START : self.camera.ROI_Y_END,
            self.camera.ROI_X_START : self.camera.ROI_X_END,
        ]

        # Detect lines
        lines = cv2.HoughLinesP(
            roi,
            rho=HOUGH_RHO,
            theta=HOUGH_THETA,
            threshold=HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LINE_LEN,
            maxLineGap=HOUGH_MAX_LINE_GAP,
        )

        debug_frame = frame.copy()
        cv2.rectangle(
            debug_frame,
            (self.camera.ROI_X_START, self.camera.ROI_Y_START),
            (self.camera.ROI_X_END, self.camera.ROI_Y_END),
            (0, 255, 0),
            2,
        )

        left_x_bottoms = []
        right_x_bottoms = []

        # Process lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x1_orig = x1 + self.camera.ROI_X_START
                y1_orig = y1 + self.camera.ROI_Y_START
                x2_orig = x2 + self.camera.ROI_X_START
                y2_orig = y2 + self.camera.ROI_Y_START

                cv2.line(
                    debug_frame, (x1_orig, y1_orig), (x2_orig, y2_orig), (0, 0, 255), 2
                )

                if x2_orig != x1_orig:
                    slope = (y2_orig - y1_orig) / (x2_orig - x1_orig)
                    intercept = y1_orig - slope * x1_orig
                    x_bottom = (self.camera.height - 1 - intercept) / slope

                    if slope < -MIN_SLOPE:
                        left_x_bottoms.append(x_bottom)
                        cv2.line(
                            debug_frame,
                            (x1_orig, y1_orig),
                            (x2_orig, y2_orig),
                            (255, 0, 0),
                            2,
                        )
                    elif slope > MIN_SLOPE:
                        right_x_bottoms.append(x_bottom)
                        cv2.line(
                            debug_frame,
                            (x1_orig, y1_orig),
                            (x2_orig, y2_orig),
                            (0, 255, 255),
                            2,
                        )

        # Compute lane positions
        left_x = np.mean(left_x_bottoms) if left_x_bottoms else None
        right_x = np.mean(right_x_bottoms) if right_x_bottoms else None

        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
        elif left_x is not None:
            lane_center = left_x + self.camera.LANE_WIDTH / 2
        elif right_x is not None:
            lane_center = right_x - self.camera.LANE_WIDTH / 2
        else:
            lane_center = self.previous_lane_center

        # Smooth lane center
        self.LANE_CENTER_HISTORY.append(lane_center)
        if len(self.LANE_CENTER_HISTORY) > HISTORY_LEN:
            self.LANE_CENTER_HISTORY.pop(0)
        lane_center_avg = np.mean(self.LANE_CENTER_HISTORY)

        # Compute error
        error = lane_center_avg - FIXED_CENTER

        # Determine steering command
        if abs(error) < THRESHOLD_SLOW:
            steering_command = "center"
        elif error > THRESHOLD_SHARP:
            steering_command = "sharp right"
        elif error > THRESHOLD_MID:
            steering_command = "mid right"
        elif error > THRESHOLD_SLOW:
            steering_command = "slow right"
        elif error < -THRESHOLD_SHARP:
            steering_command = "sharp left"
        elif error < -THRESHOLD_MID:
            steering_command = "mid left"
        elif error < -THRESHOLD_SLOW:
            steering_command = "slow left"
        else:
            steering_command = "center"

        # Set speed based on steering command
        if steering_command == "sharp left":
            speed = 140
        else:
            speed = 160

        # Combine steering and speed into full command
        full_command = f"{steering_command} {speed}"

        self.previous_lane_center = lane_center_avg

        # Send command via serial
        self.ser.send(full_command)
        
        # Visualization
        cv2.line(
            debug_frame,
            (int(lane_center_avg), 0),
            (int(lane_center_avg), self.camera.height),
            (0, 255, 255),
            2,
        )
        cv2.line(
            debug_frame,
            (FIXED_CENTER, 0),
            (FIXED_CENTER, self.camera.height),
            (255, 255, 0),
            2,
        )

        cv2.putText(
            debug_frame,
            f"Steering: {steering_command}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            debug_frame,
            f"Speed: {speed}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            debug_frame,
            f"Error: {int(error)} px",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 0),
            2,
        )
        return debug_frame
        
        
        