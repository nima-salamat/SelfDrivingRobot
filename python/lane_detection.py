import cv2
import numpy as np
from collections import deque
import time
from config import BLUR_KERNEL


class CrosswalkDetector:
    def __init__(
        self,
        hough_rho=1,
        hough_theta=np.pi / 180,
        hough_threshold=50,
        hough_min_line_length=15,
        hough_max_line_gap=100,
        horizontal_angle_threshold=25,
        vertical_angle_threshold=20,
        min_horiz=3,
        min_vert=3,
        min_line_length=10,
        max_line_length=200,
        spacing_tolerance=0.5,
        no_detection_threshold=1,
        history_length=15,
        min_history_detections=5,
        debug=False,
        ser=None,
    ):
        self.hough_rho = hough_rho
        self.hough_theta = hough_theta
        self.hough_threshold = hough_threshold
        self.hough_min_line_length = hough_min_line_length
        self.hough_max_line_gap = hough_max_line_gap
        self.horizontal_angle_threshold = horizontal_angle_threshold
        self.vertical_angle_threshold = vertical_angle_threshold
        self.min_horiz = min_horiz
        self.min_vert = min_vert
        self.min_line_length = min_line_length
        self.max_line_length = max_line_length
        self.spacing_tolerance = spacing_tolerance
        self.no_detection_threshold = no_detection_threshold
        self.history_length = history_length
        self.min_history_detections = min_history_detections
        self.detection_history = deque(maxlen=history_length)
        self.debug = debug
        self.ser = ser
        self.crosswalk_sent = False
        self.no_detection_count = 0
        self.time_ = None

    def preprocess_frame(self, frame, roi):
        x, y, w, h = roi
        cropped = frame[y : y + h, x : x + w]
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        blur = cv2.GaussianBlur(mask, (BLUR_KERNEL, BLUR_KERNEL), 0)
        return cropped, blur

    def detect_edges(self, mask):
        return cv2.Canny(mask, 50, 255)

    def detect_lines(self, edges):
        return cv2.HoughLinesP(
            edges,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap,
        )

    def filter_lines_by_angle(self, lines):
        horizontal_lines, vertical_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if self.min_line_length < length < self.max_line_length:
                    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                    if abs(angle) < self.horizontal_angle_threshold:
                        horizontal_lines.append((x1, y1, x2, y2))
                    elif abs(abs(angle) - 90) < self.vertical_angle_threshold:
                        vertical_lines.append((x1, y1, x2, y2))
        return horizontal_lines, vertical_lines

    def check_parallel_lines(self, lines, is_horizontal=True):
        if is_horizontal:
            coords = [min(y1, y2) for _, y1, _, y2 in lines]
        else:
            coords = [min(x1, x2) for x1, _, x2, _ in lines]
        coords.sort()
        if len(coords) < 2:
            return False
        spacings = [coords[i + 1] - coords[i] for i in range(len(coords) - 1)]
        if not spacings:
            return False
        mean_spacing = np.mean(spacings)
        std_spacing = np.std(spacings)
        return std_spacing < mean_spacing * self.spacing_tolerance

    def is_crosswalk(self, horizontal_lines, vertical_lines):
        horiz_parallel = self.check_parallel_lines(horizontal_lines, is_horizontal=True)
        vert_parallel = self.check_parallel_lines(vertical_lines, is_horizontal=False)
        return (len(horizontal_lines) >= self.min_horiz and horiz_parallel) or (
            len(vertical_lines) >= self.min_vert and vert_parallel
        )

    def detect(self, frame):
        # Preprocess image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        _, white_lines = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(white_lines, CANNY_LOW, CANNY_HIGH)

        # Apply ROI extraction to edges
        roi = edges[
            self.camera.ROI_Y_START : self.camera.ROI_Y_END,
            self.camera.ROI_X_START : self.camera.ROI_X_END,
        ]

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(
            roi,
            rho=HOUGH_RHO,
            theta=HOUGH_THETA,
            threshold=HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LINE_LEN,
            maxLineGap=HOUGH_MAX_LINE_GAP,
        )

        debug_frame = frame.copy() if self.debug else None
        if self.debug:
            cv2.rectangle(
                debug_frame,
                (self.camera.ROI_X_START, self.camera.ROI_Y_START),
                (self.camera.ROI_X_END, self.camera.ROI_Y_END),
                (0, 255, 0),
                2,
            )

        left_lines = []
        right_lines = []

        # Process detected lines and categorize them
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Convert to full-frame coordinates
                x1_orig = x1 + self.camera.ROI_X_START
                y1_orig = y1 + self.camera.ROI_Y_START
                x2_orig = x2 + self.camera.ROI_X_START
                y2_orig = y2 + self.camera.ROI_Y_START

                if x2_orig != x1_orig:
                    slope = (y2_orig - y1_orig) / (x2_orig - x1_orig)
                    if abs(slope) >= MIN_SLOPE:
                        # Determine the bottom x-coordinate (higher y-value)
                        x_bottom = x1_orig if y1_orig > y2_orig else x2_orig
                        line_data = (x1_orig, y1_orig, x2_orig, y2_orig, x_bottom)
                        if slope < 0:
                            left_lines.append(line_data)
                        else:
                            right_lines.append(line_data)

        # Select the leftmost line for each category
        leftmost_left_line = None
        leftmost_right_line = None

        if left_lines:
            # Leftmost left line: smallest x-coordinate at bottom
            leftmost_left_line = min(left_lines, key=lambda l: l[4])  # l[4] is x_bottom

        if right_lines:
            # Leftmost right line: smallest x-coordinate at bottom
            leftmost_right_line = min(right_lines, key=lambda l: l[4])  # l[4] is x_bottom

        # Get points for the selected lines
        left_lane_points = (
            self.get_points_along_line(
                leftmost_left_line[0],
                leftmost_left_line[1],
                leftmost_left_line[2],
                leftmost_left_line[3],
            )
            if leftmost_left_line
            else []
        )
        right_lane_points = (
            self.get_points_along_line(
                leftmost_right_line[0],
                leftmost_right_line[1],
                leftmost_right_line[2],
                leftmost_right_line[3],
            )
            if leftmost_right_line
            else []
        )

        # Fit polynomials (x = a*y^2 + b*y + c)
        left_fit = (
            np.polyfit(
                [p[1] for p in left_lane_points], [p[0] for p in left_lane_points], 2
            )
            if left_lane_points
            else None
        )
        right_fit = (
            np.polyfit(
                [p[1] for p in right_lane_points], [p[0] for p in right_lane_points], 2
            )
            if right_lane_points
            else None
        )

        # Evaluate polynomials at the bottom of the frame
        y_eval = self.camera.height - 1
        x_left = (
            left_fit[0] * y_eval**2 + left_fit[1] * y_eval + left_fit[2]
            if left_fit is not None
            else None
        )
        x_right = (
            right_fit[0] * y_eval**2 + right_fit[1] * y_eval + right_fit[2]
            if right_fit is not None
            else None
        )

        # Compute lane center
        if x_left is not None and x_right is not None:
            lane_center = (x_left + x_right) / 2
        elif x_left is not None:
            lane_center = x_left + self.camera.LANE_WIDTH / 2
        elif x_right is not None:
            lane_center = x_right - self.camera.LANE_WIDTH / 2
        else:
            lane_center = self.previous_lane_center
            full_command = f"command {ANGLE_MID_RIGHT} 120"
            self.ser.send(full_command)
            return frame

        # Moving average
        self.LANE_CENTER_HISTORY.append(lane_center)
        if len(self.LANE_CENTER_HISTORY) > HISTORY_LEN:
            self.LANE_CENTER_HISTORY.pop(0)
        lane_center_avg = np.mean(self.LANE_CENTER_HISTORY)

        # Compute error
        error = lane_center_avg - FIXED_CENTER

        # Determine steering command
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
        self.previous_lane_center = lane_center_avg

        # Debug visualization
        if self.debug and debug_frame is not None:
            # Draw selected lines
            if leftmost_left_line:
                cv2.line(
                    debug_frame,
                    (leftmost_left_line[0], leftmost_left_line[1]),
                    (leftmost_left_line[2], leftmost_left_line[3]),
                    (255, 0, 0),
                    2,
                )
            if leftmost_right_line:
                cv2.line(
                    debug_frame,
                    (leftmost_right_line[0], leftmost_right_line[1]),
                    (leftmost_right_line[2], leftmost_right_line[3]),
                    (0, 255, 255),
                    2,
                )

            # Draw fitted polynomials
            if left_fit is not None:
                ploty = np.linspace(
                    0, self.camera.height - 1, self.camera.height, dtype=int
                )
                leftx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
                for i in range(len(ploty)):
                    if 0 <= int(leftx[i]) < self.camera.width:
                        cv2.circle(
                            debug_frame, (int(leftx[i]), ploty[i]), 1, (255, 0, 0), 1
                        )
            if right_fit is not None:
                ploty = np.linspace(
                    0, self.camera.height - 1, self.camera.height, dtype=int
                )
                rightx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
                for i in range(len(ploty)):
                    if 0 <= int(rightx[i]) < self.camera.width:
                        cv2.circle(
                            debug_frame, (int(rightx[i]), ploty[i]), 1, (0, 255, 255), 1
                        )

            # Draw lane center and fixed center
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

        return frame
