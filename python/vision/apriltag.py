import cv2
from config import AT_TOP_ROI, AT_BOTTOM_ROI, AT_LEFT_ROI, AT_RIGHT_ROI

class ApriltagDetector:
    def __init__(self):
        # Define the dictionary for ArUco markers (simulated AprilTags with ArUco markers)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def detect(self, frame) -> list:
        """
        Detects ArUco markers in the given frame (simulated AprilTags with ArUco markers).

        Args:
        - frame: Input image (frame from camera).

        Returns:
        - List of detected tags, each containing the tag ID and the bounding box coordinates.
        """
        # Convert frame to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers in the grayscale image
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        detected_tags = []
        
        # If any markers are detected
        if len(corners) > 0:
            for i in range(len(corners)):
                # Get the corner coordinates of the marker
                corner = corners[i][0]
                
                # Calculate the bounding box of the marker (min and max x, y coordinates)
                min_x = min(corner[:, 0])
                max_x = max(corner[:, 0])
                min_y = min(corner[:, 1])
                max_y = max(corner[:, 1])

                # Check if the marker's bounding box is within the defined ROI
                if AT_LEFT_ROI <= min_x / frame.shape[1] <= AT_RIGHT_ROI and AT_TOP_ROI <= min_y / frame.shape[0] <= AT_BOTTOM_ROI:
                    detected_tags.append({
                        'id': ids[i][0],  # Tag ID
                        'corners': corner,
                        'center': [(min_x + max_x) / 2, (min_y + max_y) / 2],  # Center of the marker
                    })

                    # Draw the detected marker and its ID on the frame
                    cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.putText(frame, f"ID: {ids[i][0]}", 
                                (int((min_x + max_x) / 2), int((min_y + max_y) / 2)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return detected_tags
