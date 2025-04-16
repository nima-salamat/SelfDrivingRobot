import cv2
import numpy as np
from config import BLUR_KERNEL
import math


class CrosswalkDetector:
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
            edges, 1, np.pi / 180, 50, minLineLength=15, maxLineGap=100
        )

    def filter_lines_by_angle(self, lines):
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(angle) < 25:
                    if math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) > 50:
                        if (x2 + x1) / 2 < 50:
                            horizontal_lines.append((x1, y1, x2, y2))

        return horizontal_lines

    def is_crosswalk(self, horizontal_lines):
        return len(horizontal_lines) >= 1

    def detect(self, frame, roi):
        x, y, w, h = roi
        cropped, mask = self.preprocess_frame(frame, roi)
        edges = self.detect_edges(mask)
        lines = self.detect_lines(edges)
        horizontal_lines = self.filter_lines_by_angle(lines)

        return self.is_crosswalk(horizontal_lines)
