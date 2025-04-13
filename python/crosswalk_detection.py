import cv2
import numpy as np
from collections import deque
from config import BLUR_KERNEL
import time
class CrosswalkDetector:
    def __init__(
        self,
        no_detection_threshold=1,
        history_length=5,
        min_history_detections=3,
        debug=False,
    ):
        self.crosswalk_sent = False
        self.no_detection_count = 0
        self.no_detection_threshold = no_detection_threshold
        self.history_length = history_length
        self.min_history_detections = min_history_detections
        self.detection_history = deque(maxlen=history_length)
        self.debug = debug  # If True, draw lines, text, etc.

    def preprocess_frame(self, frame, roi):
        x, y, w, h = roi
        cropped = frame[y : y + h, x : x + w]
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        return cropped, blur

    def detect_edges(self, mask):
        return cv2.Canny(mask, 50, 255)
        
    def detect_lines(self, edges):
        return cv2.HoughLinesP(
            edges, 1, np.pi / 180, 30, minLineLength=10, maxLineGap=100
        )

    def filter_lines_by_angle(self, lines):
        horizontal_lines, vertical_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(angle) < 30:
                    horizontal_lines.append((x1, y1, x2, y2))
                elif abs(abs(angle) - 90) < 30:
                    vertical_lines.append((x1, y1, x2, y2))
        return horizontal_lines, vertical_lines

    def is_crosswalk(self, horizontal_lines, vertical_lines, min_horiz=3, min_vert=2):
        return (len(horizontal_lines) >= min_horiz) or (len(vertical_lines) >= min_vert)

    def detect(self, frame, roi, ser):
        time_ = None
        x, y, w, h = roi
        cropped, mask = self.preprocess_frame(frame, roi)
        edges = self.detect_edges(mask)
        lines = self.detect_lines(edges)
        horizontal_lines, vertical_lines = self.filter_lines_by_angle(lines)

        # Update detection history
        current_detection = self.is_crosswalk(horizontal_lines, vertical_lines)
        self.detection_history.append(current_detection)
        confirmed_detection = sum(self.detection_history) >= self.min_history_detections

        # Message handling
        if confirmed_detection:
            if not self.crosswalk_sent:
                time_ = time.time()
                ser.send("crosswalk")
                print("crosswalk")
                self.crosswalk_sent = True
            self.no_detection_count = 0
        else:
            self.no_detection_count += 1
            if self.no_detection_count >= self.no_detection_threshold:
                self.crosswalk_sent = False

        # Draw only if debug is enabled
        if self.debug:
            # ROI box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Horizontal lines - green
            for lx1, ly1, lx2, ly2 in horizontal_lines:
                cv2.line(frame, (x + lx1, y + ly1), (x + lx2, y + ly2), (0, 255, 0), 2)

            # Vertical lines - blue
            for vx1, vy1, vx2, vy2 in vertical_lines:
                cv2.line(frame, (x + vx1, y + vy1), (x + vx2, y + vy2), (255, 0, 0), 2)

            # Status text
            status_text = (
                "Crosswalk Detected" if confirmed_detection else "No Crosswalk"
            )
            cv2.putText(
                frame,
                status_text,
                (50, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )

        return frame, confirmed_detection, time_
