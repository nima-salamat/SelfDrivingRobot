import cv2
import numpy as np
from typing import Tuple, Dict
from cv2 import aruco
import math
from arduino_serial import send_command
import logging
from config import current_angle
logger = logging.getLogger()
from config import LAST_TAG

class VisionProcessor:
    """Handles lane, crosswalk, AprilTag, and traffic light detection."""
    global LAST_TAG

    def __init__(self, config: dict):
        self.config = config
        self.last_steering_angle = current_angle  # Default: straight
        self.apriltag_detector = None
         # Initialize ArUco for AprilTag detection
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.aruco_params = aruco.DetectorParameters()

    def detect_lines(self, frame: np.ndarray) -> Tuple[float, np.ndarray, Dict[str, float]]:
        global LAST_TAG

        height, width = frame.shape[:2]
        frame_center = width // 2
        april_tags_info = {'count': 0, 'ids': []}  # Initialize AprilTag info
        # Initialize defaults
        steering_angle = self.last_steering_angle
        line_center_x = frame_center
        target_x = frame_center
        error = 0
        right_lines_info = []
        left_lines_info = []
        cw_lines_info = []
        at_tags_info = []
        tl_color = "None"
        longest_line_length = 0
        lane_type = "none"

        # Right lane ROI
        top_rl = int(height * self.config['ROI_TOP_RL'])
        bottom_rl = int(height * self.config['ROI_BOTTOM_RL'])
        left_rl = int(width * self.config['ROI_LEFT_RL'])
        right_rl = int(width * self.config['ROI_RIGHT_RL'])
        #print(f"ROI RL: top={top_rl}, bottom={bottom_rl}, left={left_rl}, right={right_rl}")
        roi_frame_rl = frame[top_rl:bottom_rl, left_rl:right_rl]

        # Left lane ROI
        top_ll = int(height * self.config['ROI_TOP_LL'])
        bottom_ll = int(height * self.config['ROI_BOTTOM_LL'])
        left_ll = int(width * self.config['ROI_LEFT_LL'])
        right_ll = int(width * self.config['ROI_RIGHT_LL'])
        #print(f"ROI LL: top={top_ll}, bottom={bottom_ll}, left={left_ll}, right={right_ll}")
        roi_frame_ll = frame[top_ll:bottom_ll, left_ll:right_ll]

        # Crosswalk ROI
        top_cw = int(height * self.config['ROI_TOP_CW'])
        bottom_cw = int(height * self.config['ROI_BOTTOM_CW'])
        left_cw = int(width * self.config['ROI_LEFT_CW'])
        right_cw = int(width * self.config['ROI_RIGHT_CW'])
        #print(f"ROI CW: top={top_cw}, bottom={bottom_cw}, left={left_cw}, right={right_cw}")
        roi_frame_cw = frame[top_cw:bottom_cw, left_cw:right_cw]

        # AprilTag ROI
        top_at = int(height * self.config['ROI_TOP_AT'])
        bottom_at = int(height * self.config['ROI_BOTTOM_AT'])
        left_at = int(width * self.config['ROI_LEFT_AT'])
        right_at = int(width * self.config['ROI_RIGHT_AT'])
        #print(f"ROI AT: top={top_at}, bottom={bottom_at}, left={left_at}, right={right_at}")
        roi_frame_at = frame[top_at:bottom_at, left_at:right_at]

        # Traffic light ROI
        top_tl = int(height * self.config['ROI_TOP_TL'])
        bottom_tl = int(height * self.config['ROI_BOTTOM_TL'])
        left_tl = int(width * self.config['ROI_LEFT_TL'])
        right_tl = int(width * self.config['ROI_RIGHT_TL'])
        #print(f"ROI TL: top={top_tl}, bottom={bottom_tl}, left={left_tl}, right={right_tl}")
        roi_frame_tl = frame[top_tl:bottom_tl, left_tl:right_tl]

        # Process right lane
        gray_rl = cv2.cvtColor(roi_frame_rl, cv2.COLOR_RGB2GRAY)
        edge_rl = cv2.Canny(gray_rl, self.config['CANNY_LOW'], self.config['CANNY_HIGH'])
        edge_dilated_rl = cv2.dilate(edge_rl, np.ones((3, 3), np.uint8), iterations=1)
        lines_rl = cv2.HoughLinesP(edge_dilated_rl, 1, np.pi/180, self.config['HOUGH_THRESHOLD'],
                                   minLineLength=self.config['MIN_LINE_LENGTH'], maxLineGap=self.config['MAX_LINE_GAP'])

        center_right_x = right_rl + 10
        if lines_rl is not None:
            right_lines = []
            roi_height_rl = roi_frame_rl.shape[0]
            for line in lines_rl:
                for x1, y1, x2, y2 in line:
                    if x2 != x1:
                        slope = (y2 - y1) / (x2 - x1)
                        if abs(slope) >.3:
                            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                            x_bottom = x1 + (roi_height_rl - y1) / slope if abs(slope) > 1e-5 else x1
                            x_bottom = max(0, min(right_rl - left_rl - 1, x_bottom))
                            x_bottom_full = x_bottom + left_rl
                            if x_bottom_full >= frame_center:
                                right_lines.append((x1, y1, x2, y2, x_bottom, length))
                                right_lines_info.append((x1, y1, x2, y2, length))

            if right_lines:
                x_bottom_sum = sum(line[4] + left_rl for line in right_lines)
                center_right_x = x_bottom_sum / len(right_lines)
                longest_line_length = max(longest_line_length, int(max(line[5] for line in right_lines)))

        # Process left lane
        gray_ll = cv2.cvtColor(roi_frame_ll, cv2.COLOR_RGB2GRAY)
        edge_ll = cv2.Canny(gray_ll, self.config['CANNY_LOW'], self.config['CANNY_HIGH'])
        edge_dilated_ll = cv2.dilate(edge_ll, np.ones((3, 3), np.uint8), iterations=1)
        lines_ll = cv2.HoughLinesP(edge_dilated_ll, 1, np.pi/180, self.config['HOUGH_THRESHOLD'],
                                   minLineLength=self.config['MIN_LINE_LENGTH'], maxLineGap=self.config['MAX_LINE_GAP'])

        center_left_x = left_ll - 10
        if lines_ll is not None:
            left_lines = []
            roi_height_ll = roi_frame_ll.shape[0]
            for line in lines_ll:
                for x1, y1, x2, y2 in line:
                    if x2 != x1:
                        slope = (y2 - y1) / (x2 - x1)
                        if abs(slope) > 0.3:
                            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                            x_bottom = x1 + (roi_height_ll - y1) / slope if abs(slope) > 1e-5 else x1
                            x_bottom = max(0, min(right_ll - left_ll - 1, x_bottom))
                            x_bottom_full = x_bottom + left_ll
                            if x_bottom_full <= frame_center:
                                left_lines.append((x1, y1, x2, y2, x_bottom, length))
                                left_lines_info.append((x1, y1, x2, y2, length))

            if left_lines:
                x_bottom_sum = sum(line[4] + left_ll for line in left_lines)
                center_left_x = x_bottom_sum / len(left_lines)
                longest_line_length = max(longest_line_length, int(max(line[5] for line in left_lines)))

        # Process alk
        gray_cw = cv2.cvtColor(roi_frame_cw, cv2.COLOR_RGB2GRAY)
        edge_cw = cv2.Canny(gray_cw, self.config['CANNY_LOW'], self.config['CANNY_HIGH'])
        #edge_dilated_cw = cv2.dilate(edge_cw, np.ones((5, 5), np.uint8), iterations=1)
        lines_cw = cv2.HoughLinesP(edge_cw, 1, np.pi/180, self.config['HOUGH_THRESHOLD'],
                                   minLineLength=20, maxLineGap=self.config['MAX_LINE_GAP'])

        if lines_cw is not None:
            for line in lines_cw:
                for x1, y1, x2, y2 in line:
                    if x2 != x1:
                        slope = (y2 - y1) / (x2 - x1)
                        if abs(slope) >2:
                            length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                            if length > 7:
                                cw_lines_info.append((x1, y1, x2, y2, length))

# Process AprilTags
        
        gray_at = cv2.cvtColor(roi_frame_at, cv2.COLOR_RGB2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray_at, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            aruco.drawDetectedMarkers(roi_frame_at, corners, ids)
            if len(ids) > 0:  # Check if at least one marker is detected
                LAST_TAG = ids[0][0]  # Get the ID of the first detected marker
                logger.info("&&&&&&&&&&&&&& 1 &&&&&&&&&&&",LAST_TAG)
                # Use last_tag as needed, e.g., store it in an instance variable or return it
                LAST_TAG  # Example: Store in instance variable
                # Calculate area for each tag
            for i, corner in enumerate(corners):
                # Corner is a list of 4 points: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
                points = corner[0].astype(np.int32)
                
                # Calculate polygon area using the shoelace formula
                area = 0.5 * abs(
                    sum(points[j][0] * points[(j + 1) % 4][1] for j in range(4)) -
                    sum(points[j][1] * points[(j + 1) % 4][0] for j in range(4))
                )
            logger.info( "=====================",area) 
            april_tags_info['count'] = len(ids)
            april_tags_info['ids'] = ids.flatten().tolist()
            if(area>2000):
                send_command(b'STOP\r\n')
            logger.info("------------------------------------------------",april_tags_info['ids'])
            #LAST_TAG=april_tags_info['ids']
            logger.info( "===========  2  ==========",LAST_TAG) 

        else:
            # send_command(b'F\r\n')
            logger.info("F341")

        # Place AprilTag ROI back into frame for visualization
        frame[top_at:bottom_at, left_at:right_at] = roi_frame_at

        # Process traffic light
        hsv_tl = cv2.cvtColor(roi_frame_tl, cv2.COLOR_RGB2HSV)
        # Red: Two ranges to handle hue wrap-around
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([50, 255, 255])
        red_lower2 = np.array([150, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        # Green
        green_lower = np.array([25, 100, 100])
        green_upper = np.array([95, 255, 255])
        # Yellow
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        mask_red1 = cv2.inRange(hsv_tl, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv_tl, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv_tl, green_lower, green_upper)
        mask_yellow = cv2.inRange(hsv_tl, yellow_lower, yellow_upper)

        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        yellow_pixels = cv2.countNonZero(mask_yellow)

        if red_pixels > 250:
            tl_color = "Red"
        elif green_pixels >  6:
            tl_color = "Green"
        elif yellow_pixels > max(red_pixels, green_pixels, 3):
            tl_color = "Yellow"
        else:
            tl_color = "None"
        #print(f"Traffic light color: {tl_color}")

        # Steering logic
        right_detected = len(right_lines_info) > 0
        left_detected = len(left_lines_info) > 0
        lane_type = "both" if right_detected and left_detected else "right" if right_detected else "left" if left_detected else "none"
        line_center_x = (center_right_x + center_left_x) / 2
        target_x = frame_center
        error = line_center_x - target_x

        if not right_detected and not left_detected:
            steering_angle = 145
            #print(f"None - Steering angle: {steering_angle:.2f}, Error: {error:.2f}, Midpoint: {line_center_x:.2f}, R Center: {center_right_x:.2f}, L Center: {center_left_x:.2f}, CW Lines: {len(cw_lines_info)}, AT Tags: {len(at_tags_info)}, TL Color: {tl_color}")
        else:
            steering_angle = current_angle + 0.4 * error
            steering_angle = max(55, min(145, steering_angle))
            #print(f"{lane_type.capitalize()} - Steering angle: {steering_angle:.2f}, Error: {error:.2f}, Midpoint: {line_center_x:.2f}, R Center: {center_right_x:.2f}, L Center: {center_left_x:.2f}, CW Lines: {len(cw_lines_info)}, AT Tags: {len(at_tags_info)}, TL Color: {tl_color}")

        self.last_steering_angle = steering_angle

        # Draw visualization
        lanes_frame = frame.copy()
        # Right ROI (yellow)
        cv2.rectangle(lanes_frame, (left_rl, top_rl), (right_rl, bottom_rl), (0, 255, 255), 2)
        cv2.putText(lanes_frame, "RL", (left_rl + 5, top_rl + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # Left ROI (cyan)
        cv2.rectangle(lanes_frame, (left_ll, top_ll), (right_ll, bottom_ll), (255, 255, 0), 2)
        cv2.putText(lanes_frame, "LL", (left_ll + 5, top_ll + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # Crosswalk ROI (green)
        cv2.rectangle(lanes_frame, (left_cw, top_cw), (right_cw, bottom_cw), (0, 255, 0), 2)
        cv2.putText(lanes_frame, "CW", (left_cw + 5, top_cw + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
         # AprilTag ROI (green)
        cv2.rectangle(lanes_frame, (left_at, top_at), (right_at, bottom_at), (0, 255, 0), 2)
        cv2.putText(lanes_frame, "AT", (left_at + 5, top_at + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # Traffic light ROI (magenta)
        cv2.rectangle(lanes_frame, (left_tl, top_tl), (right_tl, bottom_tl), (255, 0, 255), 2)
        cv2.putText(lanes_frame, "TL", (left_tl + 5, top_tl + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # Draw lane center lines
        if right_detected:
            cv2.line(lanes_frame, (int(center_right_x), top_rl), (int(center_right_x), bottom_rl), (255, 0, 0), 2)
        else:
            cv2.line(lanes_frame, (int(center_right_x), top_rl), (int(center_right_x), bottom_rl), (255, 0, 0), 1, cv2.LINE_AA)
        if left_detected:
            cv2.line(lanes_frame, (int(center_left_x), top_ll), (int(center_left_x), bottom_ll), (0, 0, 255), 2)
        else:
            cv2.line(lanes_frame, (int(center_left_x), top_ll), (int(center_left_x), bottom_ll), (0, 0, 255), 1, cv2.LINE_AA)

        # Draw crosswalk lines
        for line in cw_lines_info:
            x1, y1, x2, y2, _ = line
            cv2.line(lanes_frame, (x1 + left_cw, y1 + top_cw), (x2 + left_cw, y2 + top_cw), (255, 255, 255), 2)

        # Draw AprilTags
        for tag in at_tags_info:
            tag_id, corners = tag
            corners = [(int(c[0]) + left_at, int(c[1]) + top_at) for c in corners]
            for i in range(4):
                cv2.line(lanes_frame, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)
            center_x = int(sum(c[0] for c in corners) / 4)
            center_y = int(sum(c[1] for c in corners) / 4)
            cv2.putText(lanes_frame, f"ID: {tag_id}", (center_x, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw traffic light color
        cv2.putText(lanes_frame, f"Color: {tl_color}", (left_tl + 5, top_tl + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.putText(lanes_frame, f"R Lines: {len(right_lines_info)}", (left_rl + 5, top_rl + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"L Lines: {len(left_lines_info)}", (left_ll + 5, top_ll + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"CW Lines: {len(cw_lines_info)}", (left_cw + 5, top_cw + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"AT Tags: {len(at_tags_info)}", (left_at + 5, top_at + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"Len: {longest_line_length} px", (left_rl + 5, top_rl + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"Lane: {lane_type}", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(lanes_frame, f"AT Tags: {april_tags_info['count']}", (left_at + 5, top_at + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        line_info = {
            'line_center_x': line_center_x,
            'target_x': target_x,
            'error': error,
            'lane_type': lane_type,
            'steering_angle': steering_angle,
            'speed': self.current_speed if hasattr(self, 'current_speed') else 100,
            'center_right_x': center_right_x,
            'center_left_x': center_left_x,
            'cw_lines': len(cw_lines_info),
            'at_tags': len(at_tags_info),
            'tl_color': tl_color,
            'april_tags_count': april_tags_info['count'],
            'april_tags_ids': april_tags_info['ids']
        }
        return steering_angle, lanes_frame, line_info