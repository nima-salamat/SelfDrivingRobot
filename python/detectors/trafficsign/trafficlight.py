import cv2
import numpy as np


class TrafficSignDetector:
    def __init__(self, min_area=500):
        # HSV color ranges
        self.RED_LOW1 = np.array([0, 70, 50])
        self.RED_HIGH1 = np.array([10, 255, 255])
        self.RED_LOW2 = np.array([170, 70, 50])
        self.RED_HIGH2 = np.array([180, 255, 255])

        self.BLUE_LOW = np.array([100, 150, 0])
        self.BLUE_HIGH = np.array([140, 255, 255])

        self.min_area = min_area

    def detect(self, frame):
        """
        Detects red/blue traffic signs (Stop, Circular, etc.)
        Returns: frame with annotations
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create color masks
        mask_red1 = cv2.inRange(hsv, self.RED_LOW1, self.RED_HIGH1)
        mask_red2 = cv2.inRange(hsv, self.RED_LOW2, self.RED_HIGH2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_blue = cv2.inRange(hsv, self.BLUE_LOW, self.BLUE_HIGH)

        mask = cv2.bitwise_or(mask_red, mask_blue)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:  # filter small noise
                approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)

                # Detect STOP sign (octagon ~ 8 sides, red)
                if len(approx) == 8 and np.mean(hsv[y:y+h, x:x+w, 0]) < 15:
                    cv2.putText(frame, "STOP SIGN", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                # Detect circular signs (blue background usually)
                elif len(approx) > 8:
                    cv2.putText(frame, "CIRCULAR SIGN", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        return frame
