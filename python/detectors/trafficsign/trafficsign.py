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
        Detect traffic signs in a frame.
        Returns:
            - frame with annotations
            - detections: list of dicts {type, color, bbox}
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create color masks
        mask_red1 = cv2.inRange(hsv, self.RED_LOW1, self.RED_HIGH1)
        mask_red2 = cv2.inRange(hsv, self.RED_LOW2, self.RED_HIGH2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_blue = cv2.inRange(hsv, self.BLUE_LOW, self.BLUE_HIGH)

        # Combine masks
        mask = cv2.bitwise_or(mask_red, mask_blue)

        detections = []
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:  # filter small noise
                approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)

                # Determine color
                mean_hue = np.mean(hsv[y:y+h, x:x+w, 0])
                color = "red" if (mean_hue < 15 or mean_hue > 160) else "blue"

                sign_type = None

                # Detect STOP sign (octagon ~ 8 sides)
                if len(approx) == 8 and color == "red":
                    sign_type = "STOP SIGN"
                    cv2.putText(frame, sign_type, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                # Detect circular signs (speed limits, mandatory)
                elif len(approx) > 8:
                    sign_type = "CIRCULAR SIGN"
                    cv2.putText(frame, sign_type, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                if sign_type:
                    detections.append({
                        "type": sign_type,
                        "color": color,
                        "bbox": (x, y, w, h)
                    })

        return frame, detections
