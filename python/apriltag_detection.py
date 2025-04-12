import math
try:
    from apriltag import Detector
except Exception:
    print("we can import apriltag in windows :D")
import cv2
from config import TAG_LABLES, MIN_SIZE_APRILTAG


class ApriltagDetector:
    def __init__(self):
        try:
            
            self.detector = Detector()
        except Exception:
            print("we cant use apriltag.Decector :D")
    def detect(self, frame):
        try:
            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the frame
            detections = self.detector.detect(gray)

            # Find the nearest apriltag
            main_size = 0
            nearest_apriltag = None
            for detection in detections:
                # Calculate the size of the detected tag (distance between two corners)
                dot1 = detection.corners[0]
                dot2 = detection.corners[2]
                size = int(math.dist(dot1, dot2))

                if size >= MIN_SIZE_APRILTAG:
                    if size > main_size:
                        nearest_apriltag = detection
                        main_size = size

            # Show nearest apriltag
            label = "no sign"
            if nearest_apriltag is not None:
                # Draw the bounding box of the tag
                cv2.polylines(
                    frame, [nearest_apriltag.corners.astype(int)], True, (240, 130, 50), 2
                )

                # Find label and get order
                try:
                    label = TAG_LABLES[nearest_apriltag.tag_id]
                except Exception:
                    pass

            return label
        except Exception:
            return "no sign"
