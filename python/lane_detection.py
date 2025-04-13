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
    MODE,
    ANGLE_SHARP_LEFT,
    ANGLE_MID_LEFT,
    ANGLE_SLOW_LEFT,
    ANGLE_CENTER,
    ANGLE_SLOW_RIGHT,
    ANGLE_MID_RIGHT,
    ANGLE_SHARP_RIGHT,
    ANGLE_VERY_SHARP_RIGHT,
    ANGLE_VERY_SHARP_LEFT,
    THRESHOLD_VERY_SHARP,
)


class LaneDetector:
    def __init__(self, camera, ser, debug=False):
        self.camera = camera
        self.ser = ser
        self.debug = debug

        # Lane tracking variables
        self.LANE_CENTER_HISTORY = []
        self.previous_lane_center = self.camera.width // 2

    def detect(self, frame):
        # Preprocess image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

        # Apply ROI extraction to edges
        roi = edges[
            self.camera.ROI_Y_START : self.camera.ROI_Y_END,
            self.camera.ROI_X_START : self.camera.ROI_X_END,
        ]

        # Detect lines using Hough Transform (search within ROI)
        lines = cv2.HoughLinesP(
            roi,
            rho=HOUGH_RHO,
            theta=HOUGH_THETA,
            threshold=HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LINE_LEN,
            maxLineGap=HOUGH_MAX_LINE_GAP,
        )

        # Create debug frame only if needed
        debug_frame = frame.copy() if self.debug else None

        # Optionally draw ROI box for debugging
        if self.debug:
            cv2.rectangle(
                debug_frame,
                (self.camera.ROI_X_START, self.camera.ROI_Y_START),
                (self.camera.ROI_X_END, self.camera.ROI_Y_END),
                (0, 255, 0),
                2,
            )

        left_x_bottoms = []
        right_x_bottoms = []

        # Process each detected line if any
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Convert ROI coordinates to full-frame coordinates
                x1_orig = x1 + self.camera.ROI_X_START
                y1_orig = y1 + self.camera.ROI_Y_START
                x2_orig = x2 + self.camera.ROI_X_START
                y2_orig = y2 + self.camera.ROI_Y_START

                # Compute the slope only if the line is not vertical
                if x2_orig != x1_orig:
                    slope = (y2_orig - y1_orig) / (x2_orig - x1_orig)
                    # Only consider lines with sufficient slope magnitude
                    if abs(slope) >= MIN_SLOPE:
                        intercept = y1_orig - slope * x1_orig
                        # Calculate x coordinate at the bottom of the frame
                        x_bottom = (self.camera.height - 1 - intercept) / slope

                        # Group line as left or right based on slope sign
                        if slope < 0:
                            left_x_bottoms.append(x_bottom)
                            if self.debug:
                                # Draw left lane line in blue if debugging
                                cv2.line(
                                    debug_frame,
                                    (x1_orig, y1_orig),
                                    (x2_orig, y2_orig),
                                    (255, 0, 0),
                                    2,
                                )
                        else:
                            right_x_bottoms.append(x_bottom)
                            if self.debug:
                                # Draw right lane line in yellow if debugging
                                cv2.line(
                                    debug_frame,
                                    (x1_orig, y1_orig),
                                    (x2_orig, y2_orig),
                                    (0, 255, 255),
                                    2,
                                )
                else:
                    # For vertical lines, you might want to handle separately if needed.
                    pass

        # Compute lane center using available left/right measurements.
        if left_x_bottoms and right_x_bottoms:
            lane_center = (np.mean(left_x_bottoms) + np.mean(right_x_bottoms)) / 2
        elif left_x_bottoms:
            lane_center = np.mean(left_x_bottoms) + self.camera.LANE_WIDTH / 2
        elif right_x_bottoms:
            lane_center = np.mean(right_x_bottoms) - self.camera.LANE_WIDTH / 2
        else:
            lane_center = self.previous_lane_center

        # Maintain a moving average for lane center
        self.LANE_CENTER_HISTORY.append(lane_center)
        if len(self.LANE_CENTER_HISTORY) > HISTORY_LEN:
            self.LANE_CENTER_HISTORY.pop(0)
        lane_center_avg = np.mean(self.LANE_CENTER_HISTORY)

        # Compute error relative to a fixed reference (desired center)
        error = lane_center_avg - FIXED_CENTER

        # Determine steering command based on error thresholds
        if abs(error) < THRESHOLD_SLOW:
            steering_command = ANGLE_CENTER
        elif error > THRESHOLD_VERY_SHARP:
            steering_command = ANGLE_VERY_SHARP_RIGHT
        elif error > THRESHOLD_SHARP:
            steering_command = ANGLE_SHARP_RIGHT
        elif error > THRESHOLD_MID:
            steering_command = ANGLE_MID_RIGHT
        elif error > THRESHOLD_SLOW:
            steering_command = ANGLE_SLOW_RIGHT
        elif error < -THRESHOLD_SHARP:
            steering_command = ANGLE_SHARP_LEFT
        elif error < -THRESHOLD_MID:
            steering_command = ANGLE_MID_LEFT
        elif error < -THRESHOLD_SLOW:
            steering_command = ANGLE_SLOW_LEFT
        elif error < -THRESHOLD_VERY_SHARP:
            steering_command = ANGLE_VERY_SHARP_LEFT
        else:
            steering_command = ANGLE_CENTER

        # Set speed based on steering command
        speed = MODE.default.turn if "sharp" in steering_command else MODE.default.forward
        

        # Build full command string and send it via serial
        full_command = f"command {steering_command} {speed}"
        self.ser.send(full_command)
        self.previous_lane_center = lane_center_avg

        # Draw debug visualization only if enabled
        if self.debug and debug_frame is not None:
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
        else:
            # If not in debug mode, return the original frame (or optionally None)
            return frame
