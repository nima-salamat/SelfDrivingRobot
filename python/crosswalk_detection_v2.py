import cv2
import numpy as np
from collections import deque
from config import BLUR_KERNEL
import time


class CrosswalkDetector:
    def __init__(
        self,
        no_detection_threshold=1,
        history_length=15,
        min_history_detections=5,
        debug=False,
        ser=None,
    ):
        self.crosswalk_sent = False
        self.no_detection_count = 0
        self.no_detection_threshold = no_detection_threshold
        self.history_length = history_length
        self.min_history_detections = min_history_detections
        self.detection_history = deque(maxlen=history_length)
        self.debug = debug  # If True, draw lines, text, etc.
        self.time_ = None
        self.ser = ser

    def preprocess_frame(self, frame, roi):
        cropped = frame[roi[0][0] : roi[0][1], roi[1][0] : roi[1][1]]

        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        return cropped, blur

    def detect_edges(self, mask):
        return cv2.Canny(mask, 50, 255)

    def detect_lines(self, edges):
        return cv2.HoughLinesP(
            edges, 1, np.pi / 180, 50, minLineLength=15, maxLineGap=100
        )

    def filter_lines_by_angle(self, lines):
        horizontal_lines, vertical_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(angle) < 25:
                    horizontal_lines.append((x1, y1, x2, y2))
                elif abs(abs(angle) - 90) < 20:
                    if (y1+y2)/2 > 100:
                        vertical_lines.append((x1, y1, x2, y2))
        return horizontal_lines, vertical_lines

    def is_crosswalk(self, horizontal_lines, vertical_lines, min_horiz=2, min_vert=2):
        return (len(horizontal_lines) >= min_horiz) and (len(vertical_lines) >= min_vert)

    def detect(self, frame, roi):
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
                self.time_ = time.time()
                # ser.send("crosswalk")
                print("crosswalk")
                self.crosswalk_sent = True
            self.no_detection_count = 0
        else:
            self.no_detection_count += 1
            if self.no_detection_count >= self.no_detection_threshold:
                self.crosswalk_sent = False

       

        return confirmed_detection
