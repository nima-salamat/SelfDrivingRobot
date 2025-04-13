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
    ROI_ORDER,
    DETECTOR_TYPE,
)



class LaneDetector:
    """
    A lane detector that supports both LSD and HoughLinesP,
    with configurable ROI extraction order.
    """

    def __init__(self, camera, ser, debug=False):
        self.camera = camera
        self.ser = ser
        self.debug = debug

        # Smoothing of lane center
        self.lane_center_history = []
        self.previous_lane_center = self.camera.width // 2

        # If using LSD, create the detector
        if DETECTOR_TYPE.upper() == "LSD":
            self.lsd = cv2.createLineSegmentDetector()

    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess the frame by converting to grayscale and applying Gaussian blur.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        return blur

    def apply_roi(self, image: np.ndarray) -> np.ndarray:
        """
        Apply the region of interest (ROI) by cropping the image.
        """
        return image[
            self.camera.ROI_Y_START : self.camera.ROI_Y_END,
            self.camera.ROI_X_START : self.camera.ROI_X_END,
        ]

    def detect_edges(self, image: np.ndarray) -> np.ndarray:
        """
        Apply Canny edge detection on the image.
        """
        return cv2.Canny(image, CANNY_LOW, CANNY_HIGH)

    def detect_lines_hough(self, roi: np.ndarray):
        """
        Use HoughLinesP to detect line segments in the ROI.
        """
        lines = cv2.HoughLinesP(
            roi,
            rho=HOUGH_RHO,
            theta=HOUGH_THETA,
            threshold=HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LINE_LEN,
            maxLineGap=HOUGH_MAX_LINE_GAP,
        )
        return lines

    def detect_lines_lsd(self, roi: np.ndarray):
        """
        Use LSD (Line Segment Detector) to detect line segments in the ROI.
        """
        # For LSD, input should be a grayscale image; no need to do edge detection.
        lines, _ = self.lsd.detect(roi)
        return lines

    def group_lines(self, lines) -> (list, list):
        """
        Process detected line segments (from either detector)
        and group them into left and right lane candidates.
        """
        left_x_bottoms = []
        right_x_bottoms = []
        if lines is None:
            return left_x_bottoms, right_x_bottoms

        # In HoughLinesP lines shape is (N, 1, 4); LSD returns (N,1,4) too.
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Convert ROI coordinates to full-frame coordinates.
            x1_full = x1 + self.camera.ROI_X_START
            y1_full = y1 + self.camera.ROI_Y_START
            x2_full = x2 + self.camera.ROI_X_START
            y2_full = y2 + self.camera.ROI_Y_START

            if x2_full != x1_full:
                slope = (y2_full - y1_full) / (x2_full - x1_full)
                if abs(slope) >= MIN_SLOPE:
                    intercept = y1_full - slope * x1_full
                    # Calculate the x coordinate at the bottom of the frame.
                    x_bottom = (self.camera.height - 1 - intercept) / slope

                    if slope < 0:
                        left_x_bottoms.append(x_bottom)
                        if self.debug:
                            cv2.line(
                                self.debug_frame,
                                (int(x1_full), int(y1_full)),
                                (int(x2_full), int(y2_full)),
                                (255, 0, 0),
                                2,
                            )
                    else:
                        right_x_bottoms.append(x_bottom)
                        if self.debug:
                            cv2.line(
                                self.debug_frame,
                                (int(x1_full), int(y1_full)),
                                (int(x2_full), int(y2_full)),
                                (0, 255, 255),
                                2,
                            )
        return left_x_bottoms, right_x_bottoms

    def compute_lane_center(self, left_x_bottoms: list, right_x_bottoms: list) -> float:
        """
        Compute the lane center by averaging left and right lane positions.
        """
        if left_x_bottoms and right_x_bottoms:
            lane_center = (np.mean(left_x_bottoms) + np.mean(right_x_bottoms)) / 2
        elif left_x_bottoms:
            lane_center = np.mean(left_x_bottoms) + self.camera.LANE_WIDTH / 2
        elif right_x_bottoms:
            lane_center = np.mean(right_x_bottoms) - self.camera.LANE_WIDTH / 2
        else:
            lane_center = self.previous_lane_center

        self.lane_center_history.append(lane_center)
        if len(self.lane_center_history) > HISTORY_LEN:
            self.lane_center_history.pop(0)
        return np.mean(self.lane_center_history)

    def map_error_to_steering(self, error: float) -> str:
        """
        Map the lane center error to a specific steering command.
        """
        if abs(error) < THRESHOLD_SLOW:
            return ANGLE_CENTER
        elif error > THRESHOLD_VERY_SHARP:
            return ANGLE_VERY_SHARP_RIGHT
        elif error > THRESHOLD_SHARP:
            return ANGLE_SHARP_RIGHT
        elif error > THRESHOLD_MID:
            return ANGLE_MID_RIGHT
        elif error > THRESHOLD_SLOW:
            return ANGLE_SLOW_RIGHT
        elif error < -THRESHOLD_VERY_SHARP:
            return ANGLE_VERY_SHARP_LEFT
        elif error < -THRESHOLD_SHARP:
            return ANGLE_SHARP_LEFT
        elif error < -THRESHOLD_MID:
            return ANGLE_MID_LEFT
        elif error < -THRESHOLD_SLOW:
            return ANGLE_SLOW_LEFT
        else:
            return ANGLE_CENTER

    def draw_debug_info(
        self,
        debug_frame: np.ndarray,
        lane_center: float,
        error: float,
        steering_command: str,
        speed: int,
    ) -> None:
        """
        Draw debug visualizations on the frame.
        """
        # Draw the computed lane center (cyan line).
        cv2.line(
            debug_frame,
            (int(lane_center), 0),
            (int(lane_center), self.camera.height),
            (0, 255, 255),
            2,
        )
        # Draw the fixed center line (light blue).
        cv2.line(
            debug_frame,
            (FIXED_CENTER, 0),
            (FIXED_CENTER, self.camera.height),
            (255, 255, 0),
            2,
        )
        # Overlay text information.
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

    def detect(self, frame: np.ndarray) -> np.ndarray:
        """
        Process a frame to detect lanes and send commands via serial.
        Chooses preprocessing order and detector based on global flags.
        """
        if self.debug:
            self.debug_frame = frame.copy()

        # --- Determine ROI extraction order ---
        if ROI_ORDER.lower() == "before":
            # Crop ROI before any processing.
            roi_frame = self.apply_roi(frame)
            processed = self.preprocess_frame(roi_frame)
        else:
            # Process full frame and then crop ROI.
            processed_full = self.preprocess_frame(frame)
            processed = self.apply_roi(processed_full)

        # For HoughLinesP, apply Canny edge detection.
        if DETECTOR_TYPE.upper() == "HOUGH":
            roi_for_detection = self.detect_edges(processed)
        else:
            roi_for_detection = processed

        # If debugging and ROI extraction is after processing, draw ROI rectangle.
        if self.debug and ROI_ORDER.lower() == "after":
            cv2.rectangle(
                self.debug_frame,
                (self.camera.ROI_X_START, self.camera.ROI_Y_START),
                (self.camera.ROI_X_END, self.camera.ROI_Y_END),
                (0, 255, 0),
                2,
            )
        elif self.debug and ROI_ORDER.lower() == "before":
            # When ROI extraction is done first, the ROI covers the full image of "processed".
            # To map it back to the original frame, draw the ROI rectangle.
            cv2.rectangle(
                self.debug_frame,
                (self.camera.ROI_X_START, self.camera.ROI_Y_START),
                (self.camera.ROI_X_END, self.camera.ROI_Y_END),
                (0, 255, 0),
                2,
            )

        # --- Line detection ---
        if DETECTOR_TYPE.upper() == "LSD":
            lines = self.detect_lines_lsd(roi_for_detection)
        else:
            lines = self.detect_lines_hough(roi_for_detection)

        left_x_bottoms, right_x_bottoms = self.group_lines(lines)
        lane_center_avg = self.compute_lane_center(left_x_bottoms, right_x_bottoms)
        error = lane_center_avg - FIXED_CENTER

        steering_command = self.map_error_to_steering(error)
        # Reduce speed if turning sharply.
        speed = (
            MODE.default.turn
            if steering_command
            in [
                ANGLE_SHARP_LEFT,
                ANGLE_SHARP_RIGHT,
                ANGLE_VERY_SHARP_LEFT,
                ANGLE_VERY_SHARP_RIGHT,
            ]
            else MODE.default.forward
        )

        # Send command via serial.
        full_command = f"command {steering_command} {speed}"
        self.ser.send(full_command)
        self.previous_lane_center = lane_center_avg

        if self.debug:
            self.draw_debug_info(
                self.debug_frame, lane_center_avg, error, steering_command, speed
            )
            return self.debug_frame
        else:
            return frame
