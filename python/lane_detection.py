# import cv2
# import numpy as np
# from config import (
#     HOUGH_RHO,
#     HOUGH_THETA,
#     HOUGH_THRESHOLD,
#     HOUGH_MAX_LINE_GAP,
#     HOUGH_MIN_LINE_LEN,
#     BLUR_KERNEL,
#     CANNY_HIGH,
#     CANNY_LOW,
#     MIN_SLOPE,
#     HISTORY_LEN,
#     THRESHOLD_MID,
#     THRESHOLD_SHARP,
#     THRESHOLD_SLOW,
#     FIXED_CENTER,
#     MODE,
#     ANGLE_SHARP_LEFT,
#     ANGLE_MID_LEFT,
#     ANGLE_SLOW_LEFT,
#     ANGLE_CENTER,
#     ANGLE_SLOW_RIGHT,
#     ANGLE_MID_RIGHT,
#     ANGLE_SHARP_RIGHT,
#     ANGLE_VERY_SHARP_RIGHT,
#     ANGLE_VERY_SHARP_LEFT,
#     THRESHOLD_VERY_SHARP,
# )


# class LaneDetector:
#     def __init__(self, camera, ser, debug=False):
#         self.camera = camera
#         self.ser = ser
#         self.debug = debug

#         # Lane tracking variables
#         self.LANE_CENTER_HISTORY = []
#         self.previous_lane_center = self.camera.width // 2

#     def detect(self, frame):
#         # Preprocess image
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
#         _, white_lines = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
#         edges = cv2.Canny(white_lines, CANNY_LOW, CANNY_HIGH)

#         # Apply ROI extraction to edges
#         roi = edges[
#             self.camera.ROI_Y_START : self.camera.ROI_Y_END,
#             self.camera.ROI_X_START : self.camera.ROI_X_END,
#         ]

#         # Detect lines using Hough Transform (search within ROI)
#         lines = cv2.HoughLinesP(
#             roi,
#             rho=HOUGH_RHO,
#             theta=HOUGH_THETA,
#             threshold=HOUGH_THRESHOLD,
#             minLineLength=HOUGH_MIN_LINE_LEN,
#             maxLineGap=HOUGH_MAX_LINE_GAP,
#         )

#         # Create debug frame only if needed
#         debug_frame = frame.copy() if self.debug else None

#         # Optionally draw ROI box for debugging
#         if self.debug:
#             cv2.rectangle(
#                 debug_frame,
#                 (self.camera.ROI_X_START, self.camera.ROI_Y_START),
#                 (self.camera.ROI_X_END, self.camera.ROI_Y_END),
#                 (0, 255, 0),
#                 2,
#             )

#         left_x_bottoms = []
#         right_x_bottoms = []

#         # Process each detected line if any
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 # Convert ROI coordinates to full-frame coordinates
#                 x1_orig = x1 + self.camera.ROI_X_START
#                 y1_orig = y1 + self.camera.ROI_Y_START
#                 x2_orig = x2 + self.camera.ROI_X_START
#                 y2_orig = y2 + self.camera.ROI_Y_START

#                 # Compute the slope only if the line is not vertical
#                 if x2_orig != x1_orig:
#                     slope = (y2_orig - y1_orig) / (x2_orig - x1_orig)
#                     # Only consider lines with sufficient slope magnitude
#                     if abs(slope) >= MIN_SLOPE:
#                         intercept = y1_orig - slope * x1_orig
#                         # Calculate x coordinate at the bottom of the frame
#                         x_bottom = (self.camera.height - 1 - intercept) / slope

#                         # Group line as left or right based on slope sign
#                         if slope < 0:
#                             left_x_bottoms.append(x_bottom)
#                             if self.debug:
#                                 # Draw left lane line in blue if debugging
#                                 cv2.line(
#                                     debug_frame,
#                                     (x1_orig, y1_orig),
#                                     (x2_orig, y2_orig),
#                                     (255, 0, 0),
#                                     2,
#                                 )
#                         else:
#                             right_x_bottoms.append(x_bottom)
#                             if self.debug:
#                                 # Draw right lane line in yellow if debugging
#                                 cv2.line(
#                                     debug_frame,
#                                     (x1_orig, y1_orig),
#                                     (x2_orig, y2_orig),
#                                     (0, 255, 255),
#                                     2,
#                                 )
#                 else:
#                     # For vertical lines, you might want to handle separately if needed.
#                     pass

#         # Compute lane center using available left/right measurements.
#         if left_x_bottoms and right_x_bottoms:
#             lane_center = (np.mean(left_x_bottoms) + np.mean(right_x_bottoms)) / 2
#         elif left_x_bottoms:
#             lane_center = np.mean(left_x_bottoms) + self.camera.LANE_WIDTH / 2
#         elif right_x_bottoms:
#             lane_center = np.mean(right_x_bottoms) - self.camera.LANE_WIDTH / 2
#         else:
#             lane_center = self.previous_lane_center
#             full_command = f"command {ANGLE_MID_RIGHT} 120"
#             self.ser.send(full_command)
#             return frame

#         # Maintain a moving average for lane center
#         self.LANE_CENTER_HISTORY.append(lane_center)
#         if len(self.LANE_CENTER_HISTORY) > HISTORY_LEN:
#             self.LANE_CENTER_HISTORY.pop(0)
#         lane_center_avg = np.mean(self.LANE_CENTER_HISTORY)

#         # Compute error relative to a fixed reference (desired center)
#         error = lane_center_avg - FIXED_CENTER

#         # Determine steering command based on error thresholds
#         if abs(error) < THRESHOLD_SLOW:
#             steering_command = ANGLE_CENTER
#         elif error > THRESHOLD_VERY_SHARP:
#             steering_command = ANGLE_VERY_SHARP_RIGHT
#         elif error > THRESHOLD_SHARP:
#             steering_command = ANGLE_SHARP_RIGHT
#         elif error > THRESHOLD_MID:
#             steering_command = ANGLE_MID_RIGHT
#         elif error > THRESHOLD_SLOW:
#             steering_command = ANGLE_SLOW_RIGHT
#         elif error < -THRESHOLD_SHARP:
#             steering_command = ANGLE_SHARP_LEFT
#         elif error < -THRESHOLD_MID:
#             steering_command = ANGLE_MID_LEFT
#         elif error < -THRESHOLD_SLOW:
#             steering_command = ANGLE_SLOW_LEFT
#         elif error < -THRESHOLD_VERY_SHARP:
#             steering_command = ANGLE_VERY_SHARP_LEFT
#         else:
#             steering_command = ANGLE_MID_RIGHT

#         # Set speed based on steering command
#         if steering_command in [
#             ANGLE_SHARP_LEFT,
#             ANGLE_SHARP_RIGHT,
#             ANGLE_VERY_SHARP_LEFT,
#             ANGLE_VERY_SHARP_RIGHT,
#         ]:
#             speed = MODE.default.turn
#         else:
#             speed = MODE.default.forward

#         # Build full command string and send it via serial
#         full_command = f"command {steering_command} {speed}"
#         self.ser.send(full_command)
#         self.previous_lane_center = lane_center_avg

#         # Draw debug visualization only if enabled
#         if self.debug and debug_frame is not None:
#             cv2.line(
#                 debug_frame,
#                 (int(lane_center_avg), 0),
#                 (int(lane_center_avg), self.camera.height),
#                 (0, 255, 255),
#                 2,
#             )
#             cv2.line(
#                 debug_frame,
#                 (FIXED_CENTER, 0),
#                 (FIXED_CENTER, self.camera.height),
#                 (255, 255, 0),
#                 2,
#             )
#             cv2.putText(
#                 debug_frame,
#                 f"Steering: {steering_command}",
#                 (10, 30),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 1,
#                 (0, 255, 0),
#                 2,
#             )
#             cv2.putText(
#                 debug_frame,
#                 f"Speed: {speed}",
#                 (10, 60),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 1,
#                 (0, 255, 0),
#                 2,
#             )
#             cv2.putText(
#                 debug_frame,
#                 f"Error: {int(error)} px",
#                 (10, 90),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 1,
#                 (255, 255, 0),
#                 2,
#             )
#             return debug_frame
#         else:
#             # If not in debug mode, return the original frame (or optionally None)
#             return frame
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
        self.LANE_CENTER_HISTORY = []
        self.previous_lane_center = self.camera.width // 2

    def get_points_along_line(self, x1, y1, x2, y2):
        """Generate (x, y) points along a line segment from (x1, y1) to (x2, y2)."""
        points = []
        if y1 > y2:
            x1, y1, x2, y2 = x2, y2, x1, y1  # Ensure y1 <= y2
        dy = y2 - y1
        if dy == 0:  # Horizontal line
            x_start, x_end = min(x1, x2), max(x1, x2)
            for x in range(x_start, x_end + 1):
                points.append((x, y1))
        else:
            for y in range(y1, y2 + 1):
                x = x1 + (x2 - x1) * (y - y1) / dy
                points.append((int(x), y))
        return points

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
            self.get_points_along_line(leftmost_left_line[0], leftmost_left_line[1], 
                                    leftmost_left_line[2], leftmost_left_line[3])
            if leftmost_left_line
            else []
        )
        right_lane_points = (
            self.get_points_along_line(leftmost_right_line[0], leftmost_right_line[1], 
                                    leftmost_right_line[2], leftmost_right_line[3])
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