import cv2
import numpy as np
from collections import deque
from config import BLUR_KERNEL
import time


class CrosswalkDetector:
    def preprocess_frame(self, frame, roi):
        cropped = frame[roi[0][0] : roi[0][1], roi[1][0] : roi[1][1]]

        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (BLUR_KERNEL, BLUR_KERNEL), 0)
        _, white_lines = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

        return cropped, white_lines

    def detect_edges(self, mask):
        return cv2.Canny(mask, 50, 255)

    def detect_lines(self, edges):
        return cv2.HoughLinesP(
            edges, 1, np.pi / 180, 50, minLineLength=30, maxLineGap=10
        )

    def filter_lines_by_angle(self, lines):
        horizontal_lines, vertical_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(angle) < 25:
                    if (y1 + y2) / 2 > 180:
                        horizontal_lines.append((x1, y1, x2, y2))
                elif abs(abs(angle) - 90) < 20:
                    vertical_lines.append((x1, y1, x2, y2))
        return horizontal_lines, vertical_lines

    def is_crosswalk(self, horizontal_lines, vertical_lines, min_horiz=2, min_vert=2):
        return (len(horizontal_lines) >= min_horiz) and (
            len(vertical_lines) >= min_vert
        )

    def detect(self, frame, roi):
        cropped, mask = self.preprocess_frame(frame, roi)
        edges = self.detect_edges(mask)
        lines = self.detect_lines(edges)
        horizontal_lines, vertical_lines = self.filter_lines_by_angle(lines)

        # Update detection history
        current_detection = self.is_crosswalk(horizontal_lines, vertical_lines)

        return current_detection
