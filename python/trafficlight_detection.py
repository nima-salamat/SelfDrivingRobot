import cv2
import numpy as np


def region_of_interest(image, vertices):
    """
    Applies a polygonal mask to the input image based on provided vertices.

    Parameters:
        image (np.array): The input image (e.g., in HSV or BGR).
        vertices (list of tuple): Coordinates of the polygon vertices.

    Returns:
        masked_image (np.array): The image after applying the ROI mask.
    """
    mask = np.zeros_like(image)

    # For multi-channel images, create a mask with the same channel count.
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    cv2.fillPoly(mask, np.array([vertices], dtype=np.int32), ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def normalized_roi_to_vertices(normalized_roi, width, height):
    """
    Converts a normalized ROI into absolute pixel vertices.

    Parameters:
        normalized_roi (list): A 2D list [[x_min, x_max], [y_min, y_max]]
                               where each value is between 0 and 1.
        width (int): Width of the image.
        height (int): Height of the image.

    Returns:
        vertices (list of tuple): List of four vertices (top-left, top-right,
                                  bottom-right, bottom-left).
    """
    x_min = int(normalized_roi[0][0] * width)
    x_max = int(normalized_roi[0][1] * width)
    y_min = int(normalized_roi[1][0] * height)
    y_max = int(normalized_roi[1][1] * height)

    vertices = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    return vertices


class TrafficLightDetector:
    def __init__(self, width, height, normalized_roi):
        self.width = width
        self.height = height
        # normalized_roi should be in the form: [[x_min, x_max], [y_min, y_max]]
        self.normalized_roi = normalized_roi
        self.roi_vertices = normalized_roi_to_vertices(normalized_roi, width, height)

    def detect(self, frame):
        """
        Detects traffic lights in the given frame within the specified ROI.
        Converts the frame to HSV, applies a color threshold for red and green,
        and then checks for contours inside the ROI.
        """
        order = "no light"
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Draw the ROI polygon for visualization.
        cv2.polylines(
            frame,
            [np.array(self.roi_vertices, np.int32)],
            isClosed=True,
            color=(0, 255, 255),
            thickness=2,
        )

        # Extract ROI from the HSV image.
        roi_hsv = region_of_interest(hsv, self.roi_vertices)

        # Define two HSV ranges for red detection.
        lower_red1 = np.array([0, 50, 100])
        upper_red1 = np.array([25, 150, 255])
        lower_red2 = np.array([160, 50, 100])
        upper_red2 = np.array([180, 150, 255])

        mask_red1 = cv2.inRange(roi_hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(roi_hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Define HSV range for green detection.
        lower_green = np.array([30, 50, 50])
        upper_green = np.array([60, 150, 255])
        mask_green = cv2.inRange(roi_hsv, lower_green, upper_green)

        # Dilate the masks to fill gaps.
        kernel = np.ones((5, 5), np.uint8)
        dilated_mask_red = cv2.dilate(mask_red, kernel)
        dilated_mask_green = cv2.dilate(mask_green, kernel)

        # Find contours.
        contours_red, _ = cv2.findContours(
            dilated_mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        contours_green, _ = cv2.findContours(
            dilated_mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        # Filter out small contours.
        min_area = 500
        valid_contours_red = [
            cnt for cnt in contours_red if cv2.contourArea(cnt) > min_area
        ]
        valid_contours_green = [
            cnt for cnt in contours_green if cv2.contourArea(cnt) > min_area
        ]

        # Optionally, draw bounding boxes for visualization.
        for cnt in valid_contours_red:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        for cnt in valid_contours_green:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Compute total contour area to decide which light is predominant.
        area_red = sum(cv2.contourArea(cnt) for cnt in valid_contours_red)
        area_green = sum(cv2.contourArea(cnt) for cnt in valid_contours_green)

        if area_green > area_red and area_green > 0:
            order = "green light"
        elif area_red > area_green and area_red > 0:
            order = "red light"

        return frame, order
