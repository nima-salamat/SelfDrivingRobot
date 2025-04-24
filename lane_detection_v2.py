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
    def __init__(self, camera,ser, debug=False):
        self.camera = camera
        self.width = camera.width
        self.height = camera.height
        self.debug = debug
        # Steering parameters
        self.fixed_center = self.width // 2  # Image horizontal center
        self.steering_gain = 0.1  # Proportional gain for steering angle
        self.lane_width = self.width // 5  # Assume lane width is 1/5 of frame width

    def detect(self, frame, roi_=None):
        # Preprocess image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, white_lines = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        # Define right ROI if not provided
        if not roi_:
            # Right ROI: Full height, 1/5 width from right edge (x=4*width/5 to x=width)
            roi_right = np.array(
                [
                    [
                        (4 * self.width // 5, 0),
                        (self.width, 0),
                        (self.width, self.height),
                        (4 * self.width // 5, self.height),
                    ]
                ],
                dtype=np.int32,
            )
        else:
            roi_right = np.array([roi_], dtype=np.int32)

        # Create mask for right ROI
        mask_right = np.zeros_like(white_lines)
        cv2.fillPoly(mask_right, [roi_right], 255)

        # Apply mask to white_lines
        white_lines_right = cv2.bitwise_and(white_lines, white_lines, mask=mask_right)

        # Find contours in right ROI
        contours_right, _ = cv2.findContours(
            white_lines_right, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )

        # Find the largest contour in right ROI
        largest_contour_right = (
            max(contours_right, key=cv2.contourArea) if contours_right else None
        )

        # Calculate center of the largest contour
        center_right = None
        if largest_contour_right is not None:
            M = cv2.moments(largest_contour_right)
            if M["m00"] != 0:
                center_right = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # Calculate steering angle
        steering_angle = 0.0  # Default to straight
        midpoint = None

        if center_right:
            # Use right center, assume desired path is half a lane width to the left
            midpoint = (center_right[0] - self.lane_width // 2, center_right[1])
            deviation = midpoint[0] - self.fixed_center
            steering_angle = deviation * self.steering_gain
            # Clamp steering angle to [-45, 45] degrees
            steering_angle = max(min(steering_angle, 45), -45)
            error = (steering_angle * 10)
            
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
            elif error < -THRESHOLD_VERY_SHARP:
                steering_command = ANGLE_VERY_SHARP_LEFT
            elif error < -THRESHOLD_SHARP:
                steering_command = ANGLE_SHARP_LEFT
            elif error < -THRESHOLD_MID:
                steering_command = ANGLE_MID_LEFT
            elif error < -THRESHOLD_SLOW:
                steering_command = ANGLE_SLOW_LEFT
            else:
                steering_command = ANGLE_MID_RIGHT
        
            # Set speed
            if steering_command in [
                ANGLE_SHARP_LEFT,
                ANGLE_SHARP_RIGHT,
                ANGLE_VERY_SHARP_LEFT,
                ANGLE_VERY_SHARP_RIGHT,
            ]:
                speed = MODE.default.turn
            else:
                speed = MODE.default.forward
        
            full_command = f"command {steering_command} {speed}"
            self.ser.send(full_command)

        # Debug visualization on the original frame
        debug_frame = frame.copy()
        if self.debug:
            # Draw right ROI rectangle
            cv2.polylines(
                debug_frame,
                [roi_right],
                isClosed=True,
                color=(255, 0, 255),
                thickness=2,
            )  # Magenta for right ROI
            # Draw right contour
            if largest_contour_right is not None:
                cv2.drawContours(
                    debug_frame, [largest_contour_right], -1, (0, 0, 255), 2
                )  # Red for right
            # Draw right center and midpoint
            if center_right:
                cv2.circle(
                    debug_frame, center_right, 5, (255, 0, 0), -1
                )  # Blue dot for right center
            if midpoint:
                cv2.circle(
                    debug_frame, midpoint, 5, (0, 255, 255), -1
                )  # Yellow dot for midpoint
                cv2.line(
                    debug_frame,
                    (self.fixed_center, 0),
                    (self.fixed_center, self.height),
                    (255, 255, 0),
                    1,
                )  # Cyan line for fixed_center
            # Display steering angle
            cv2.putText(
                debug_frame,
                f"Steering: {steering_angle:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

        return frame


