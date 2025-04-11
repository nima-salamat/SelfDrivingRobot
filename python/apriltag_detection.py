import math
import apriltag
import cv2
from config import TAG_LABLES, MIN_SIZE_APRILTAG





detector = Detector(families="tag36h11")


def apriltag_detection(frame):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the frame
    detections = detector.detect(gray, estimate_tag_pose=False)

    # Find the nearest apriltag
    main_size = 0
    nearest_apriltag = None
    for detection in detections:
        corners = [detection.corners.astype(int)]
        dot1 = [corners[0][0][0], corners[0][0][1]]
        dot2 = [corners[0][2][0], corners[0][2][1]]
        size = int(math.dist(dot2, dot1))

            if size >= MIN_SIZE_APRILTAG:
                if size > main_size:
                    nearest_apriltag = detection
                    main_size = size

    # Show nearest apriltag
    label = "no sign"
    if nearest_apriltag is not None:
        cv2.polylines(
            frame, [nearest_apriltag.corners.astype(int)], True, (240, 130, 50), 2
        )

            # Find label and get order
            try:
                label = TAG_LABLES[nearest_apriltag.tag_id]
            except Exception:
                pass


        return label
