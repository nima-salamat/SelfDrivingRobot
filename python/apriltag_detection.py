import cv2
import math
from dt_apriltags import Detector
from config import (
    TAG_LABLES,
    MIN_SIZE_APRILTAG,
    THRESHOLD_VALUE,
    USE_ADAPTIVE_THRESHOLD,
)  


class ApriltagDetector:
    def __init__(self):
        self.detector = Detector(
            searchpath=["apriltags"],
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



        detections = self.detector.detect(
            gray, estimate_tag_pose=False
        )  

        main_size = 0
        nearest_apriltag = None
        for detection in detections:
            corners = detection.corners
            dot1 = corners[0]
            dot2 = corners[2]
            size = int(math.dist(dot1, dot2))

            if size >= MIN_SIZE_APRILTAG:
                if size > main_size:
                    nearest_apriltag = detection
                    main_size = size

        label = "no sign"
        if nearest_apriltag is not None:
            cv2.polylines(
                frame,
                [nearest_apriltag.corners.astype(int)],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2,
            )
            tag_id = nearest_apriltag.tag_id
            label = TAG_LABLES.get(tag_id, f"ID {tag_id}")

        return label
