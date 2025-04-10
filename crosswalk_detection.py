# # Import libraries
# import cv2
# import numpy as np
# from mod import region_of_interest as ROI


# # Function to detect crosswalk
# def crosswalk_detection(frame):
#     order = "no crosswalk"
#     # Convert the frame to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Apply GaussianBlur to reduce noise and help with edge detection
#     blurred = cv2.GaussianBlur(gray, (5, 5), 0)

#     # Perform edge detection using canny
#     edges = cv2.Canny(blurred, 50, 150)

#     # Define region of interests (ROI) to focus on crosswalk lines
#     height, width = gray.shape
#     roi_up = height - 30

#     roi_vertices = [(0, height), (0, roi_up), (width, roi_up), (width, height)]
#     fme = frame.copy()
#     roi, _ = ROI.region_of_interest(edges, fme, roi_vertices)

#     # Use HoughLines to detect lines in the image
#     lines = cv2.HoughLinesP(
#         roi, 1, np.pi / 180, threshold=25, minLineLength=20, maxLineGap=20
#     )

#     # Draw the detected lines on the original image
#     if lines is not None:
#         num_line = 0
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             theta = abs(y2 - y1) / abs(x2 - x1)

#             # Separate lines that are not horizontal
#             if theta >= 1:
#                 num_line += 1

#                 # debuging
#                 cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)

#         # Get order
#         if num_line > 8:
#             order = "crosswalk"
#             array_vertices = [np.array(roi_vertices, np.int32)]
#             cv2.fillPoly(frame, array_vertices, (240, 130, 50))

#         else:
#             order = "no crosswalk"

#     return order

import cv2
import numpy as np
import time
def preprocess_frame(frame, roi):
    """Crop the frame to the region of interest (ROI) and apply an HSV mask."""
    x, y, w, h = roi
    cropped = frame[y : y + h, x : x + w]
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

    # White color range (tweak if needed)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([255, 30, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    return cropped, mask


def detect_edges(mask):
    """Apply Canny edge detection."""
    return cv2.Canny(mask, 50, 150)


def detect_lines(edges):
    """Detect lines using the probabilistic Hough Transform."""
    return cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=100)


def find_steering_line(lines, roi):
    """
    Find the main guiding line (for lane steering) inside the ROI and return its x-position
    in the full frame's coordinate system.
    """
    if lines is None:
        return None

    x, _, _, _ = roi
    line_positions = []

    for line in lines:
        x1, _, x2, _ = line[0]
        midpoint_x = (x1 + x2) // 2
        line_positions.append(midpoint_x)

    if not line_positions:
        return None

    avg_x = int(np.mean(line_positions)) + x  # Convert back to full frame coordinates
    return avg_x


def determine_steering(steering_x, width):
    """Determine steering based on the detected line's position relative to the frame's center."""
    if steering_x is None:
        return "No Line Found"

    center = width // 2
    offset = steering_x - center

    if offset < -50:
        return "Turn Left"
    elif offset > 50:
        return "Turn Right"
    else:
        return "Go Straight"



def detect_crosswalk_lines(edges):
    """
    Detect lines in the edge image and filter out nearly horizontal lines.
    Returns a list of horizontal line segments.
    """
    lines = detect_lines(edges)
    horizontal_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Angle in degrees
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            # Keep lines that are nearly horizontal (within ±15°)
            if abs(angle) < 15:
                horizontal_lines.append((x1, y1, x2, y2))

    return horizontal_lines


def is_crosswalk(horizontal_lines, min_lines=3):
    """
    Simple heuristic: if there are at least 'min_lines' nearly horizontal lines
    in close proximity, consider it a crosswalk.
    """
    global cross_walk_time
    if time.time() - cross_walk_time > 5:
        cross_walk_time = time.time()
        return len(horizontal_lines) >= min_lines
    return False


cross_walk_time = 0


def process_frame_for_crosswalk(frame, roi, ser):
    """
    Detect crosswalks within the ROI and overlay the detection result on the frame.
    """
    cropped, mask = preprocess_frame(frame, roi)
    edges = detect_edges(mask)
    horizontal_lines = detect_crosswalk_lines(edges)

    # Draw the ROI rectangle for crosswalk detection
    x, y, w, h = roi
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # Draw detected horizontal lines (in green)
    for lx1, ly1, lx2, ly2 in horizontal_lines:
        cv2.line(frame, (x + lx1, y + ly1), (x + lx2, y + ly2), (0, 255, 0), 2)

    # Display the crosswalk detection result
    cross_walk_detected = is_crosswalk(horizontal_lines, min_lines=3)
    text = (
        "Crosswalk Detected"
        if cross_walk_detected
        else "No Crosswalk"
    )

    cv2.putText(frame, text, (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    if cross_walk_detected:
        ser.send("crosswalk")
        
    return frame, cross_walk_detected