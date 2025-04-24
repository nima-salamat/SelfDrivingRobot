import cv2
import math
import apriltag
from config import (
    TAG_LABLES,
    MIN_SIZE_APRILTAG,
    THRESHOLD_VALUE,
    USE_ADAPTIVE_THRESHOLD,
)


class ApriltagDetector:
    def __init__(self):
        # Initialize the apriltag detector with supported options
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11", nthreads=1, quad_decimate=1.0)
        )

    def detect(self, frame):
        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Pass grayscale image directly to the detector
        detections = self.detector.detect(gray)

        main_size = 0
        nearest_apriltag = None
        for detection in detections:
            # Get corners of the tag
            corners = detection.corners
            dot1 = corners[0]
            dot2 = corners[2]
            size = int(math.dist(dot1, dot2))

            # Filter by minimum size
            if size >= MIN_SIZE_APRILTAG:
                if size > main_size:
                    nearest_apriltag = detection
                    main_size = size

        label = "no sign"
        if nearest_apriltag is not None:
            # Draw polygon around the detected tag
            cv2.polylines(
                frame,
                [nearest_apriltag.corners.astype(int)],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2,
            )
            # Get tag ID and map to label
            tag_id = nearest_apriltag.tag_id
            label = TAG_LABLES.get(tag_id, f"ID {tag_id}")

        return label
