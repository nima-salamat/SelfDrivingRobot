import cv2
import numpy as np
from collections import deque


class CrosswalkDetector:
    def __init__(
        self, no_detection_threshold=1, history_length=15, min_history_detections=10
    ):
        # Flag to indicate a crosswalk event was already signaled.
        self.crosswalk_sent = False
        # Counter for consecutive frames with no crosswalk detection.
        self.no_detection_count = 0
        # Number of consecutive frames with no detection required to reset the event.
        self.no_detection_threshold = no_detection_threshold

        # Temporal consistency check:
        # - history_length: Number of previous frames to consider.
        # - min_history_detections: Minimum number of detections in history to confirm crosswalk.
        self.history_length = history_length
        self.min_history_detections = min_history_detections
        self.detection_history = deque(
            maxlen=history_length
        )  # Stores True/False for recent detections.

    def preprocess_frame(self, frame, roi):
        """Crop the frame to the ROI and apply an HSV mask."""
        x, y, w, h = roi
        cropped = frame[y : y + h, x : x + w]
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        # White color detection thresholds (tweak as necessary for your environment)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return cropped, mask

    def detect_edges(self, mask):
        """Apply Canny edge detection."""
        return cv2.Canny(mask, 50, 150)

    def detect_lines(self, edges):
        """Detect lines using the probabilistic Hough Transform."""
        return cv2.HoughLinesP(
            edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=100
        )

    def detect_crosswalk_horizontal_lines(self, edges):
        """
        Detect nearly horizontal lines in the ROI.
        Returns a list of horizontal line segments (ROI coordinates).
        """
        lines = self.detect_lines(edges)
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(angle) < 15:  # Keep lines that are nearly horizontal.
                    horizontal_lines.append((x1, y1, x2, y2))
        return horizontal_lines

    def detect_crosswalk_vertical_lines(self, edges):
        """
        Detect nearly vertical lines in the ROI.
        Returns a list of vertical line segments (ROI coordinates).
        """
        lines = self.detect_lines(edges)
        vertical_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                if abs(abs(angle) - 90) < 15:  # Keep lines that are nearly vertical.
                    vertical_lines.append((x1, y1, x2, y2))
        return vertical_lines

    def is_crosswalk(self, horizontal_lines, vertical_lines, min_horiz=3, min_vert=2):
        """
        Heuristic decision:
        - Consider a crosswalk detected if there are at least `min_horiz` horizontal lines,
          or at least `min_vert` vertical lines.
        """
        return (len(horizontal_lines) >= min_horiz) or (len(vertical_lines) >= min_vert)

    def process_frame(self, frame, roi, ser):
        """
        Process the frame to detect a crosswalk.
        The crosswalk event is sent only once until it disappears for a set number
        of consecutive frames (defined by no_detection_threshold).
        """
        x, y, w, h = roi
        cropped, mask = self.preprocess_frame(frame, roi)
        edges = self.detect_edges(mask)

        # Detect horizontal and vertical lines within the ROI.
        horizontal_lines = self.detect_crosswalk_horizontal_lines(edges)
        vertical_lines = self.detect_crosswalk_vertical_lines(edges)

        # Draw the ROI rectangle for visualization.
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # Draw detected horizontal lines (green).
        for lx1, ly1, lx2, ly2 in horizontal_lines:
            cv2.line(frame, (x + lx1, y + ly1), (x + lx2, y + ly2), (0, 255, 0), 2)
        # Draw detected vertical lines (blue).
        for vx1, vy1, vx2, vy2 in vertical_lines:
            cv2.line(frame, (x + vx1, y + vy1), (x + vx2, y + vy2), (255, 0, 0), 2)

        # Determine if a crosswalk is detected in the current frame.
        current_detection = self.is_crosswalk(
            horizontal_lines, vertical_lines, min_horiz=3, min_vert=2
        )
        self.detection_history.append(current_detection)

        # Temporal consistency check: Only confirm detection if at least `min_history_detections`
        # of the last `history_length` frames had detections.
        confirmed_detection = sum(self.detection_history) >= self.min_history_detections

        # Send the "crosswalk" message only when a new confirmed detection occurs.
        if confirmed_detection:
            if not self.crosswalk_sent:
                ser.send("crosswalk")
                print("crosswalk")
                self.crosswalk_sent = True
            self.no_detection_count = 0  # Reset no-detection counter.
        else:
            self.no_detection_count += 1
            if self.no_detection_count >= self.no_detection_threshold:
                self.crosswalk_sent = False

        # Display the detection status on the frame.
        status_text = "Crosswalk Detected" if confirmed_detection else "No Crosswalk"
        cv2.putText(
            frame,
            status_text,
            (50, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 255),
            2,
        )

        return frame, confirmed_detection
