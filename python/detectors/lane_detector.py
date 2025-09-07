import cv2
import numpy as np
from typing import Tuple, Dict, List
from cv2 import aruco
import math
from arduino_serial import send_command
import logging
from config import current_angle, LAST_TAG

logger = logging.getLogger(__name__)


class VisionProcessor:
    """Refactored VisionProcessor that is safe when debug=False.

    Public API:
        detector = VisionProcessor(config, mode='race', debug=True)
        steering, debug_frame_or_none, info = detector.detect(frame)
    """

    def __init__(self, config: dict, mode: str = "race", debug: bool = True):
        self.config = config
        self.mode = mode.lower()
        self.debug = bool(debug)
        self.last_steering_angle = current_angle
        self.current_speed = config.get("DEFAULT_SPEED", 100)

        # prepare aruco (AprilTag) detector once
        # use APRILTAG dictionary (if OpenCV compiled with AprilTag support)
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        except Exception:
            # fallback to a generic ArUco if APRILTAG not available
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters_create()
        except AttributeError:
            # older OpenCV
            self.aruco_params = aruco.DetectorParameters()

    # ---------------------- Public API ----------------------
    def set_mode(self, mode: str):
        self.mode = mode.lower()

    def set_debug(self, debug: bool):
        self.debug = bool(debug)

    def detect(self, frame: np.ndarray) -> Tuple[float, np.ndarray, Dict[str, object]]:
        """Main detect method.
        Returns (steering_angle, debug_frame_or_None, info_dict).
        If debug is False, debug_frame_or_None is None and no drawing occurs.
        """
        if frame is None:
            raise ValueError("Frame is None")

        # Defensive copy for drawing (only used if debug True)
        draw_frame = frame.copy() if self.debug else None

        height, width = frame.shape[:2]
        frame_center = width // 2

        # default info
        info = {
            "line_center_x": frame_center,
            "target_x": frame_center,
            "error": 0.0,
            "lane_type": "none",
            "steering_angle": self.last_steering_angle,
            "speed": self.current_speed,
            "center_right_x": frame_center,
            "center_left_x": frame_center,
            "cw_lines": 0,
            "at_tags": 0,
            "tl_color": "None",
            "april_tags_count": 0,
            "april_tags_ids": []
        }

        # get ROIs once
        rois = self._get_rois(frame.shape)

        # race mode: only detect left/right lanes
        if self.mode == "race":
            steering_angle, centers, longest = self._detect_lanes_only(frame, rois)
            info.update({
                "center_right_x": centers[0],
                "center_left_x": centers[1],
                "len": longest,
            })
            # draw if debug
            if self.debug:
                self._draw_lane_visuals(draw_frame, rois, centers, lane_type="both")

        # city mode: full processing
        else:
            # for apriltag, provide draw_roi so aruco markers can be drawn only when debug True
            steering_angle, centers, cw_count, at_info, tl_color, longest = self._detect_city(frame, rois, draw_frame)
            info.update({
                "center_right_x": centers[0],
                "center_left_x": centers[1],
                "cw_lines": cw_count,
                "at_tags": len(at_info),
                "tl_color": tl_color,
                "april_tags_count": len(at_info),
                "april_tags_ids": [t[0] for t in at_info],
                "len": longest,
            })
            if self.debug:
                self._draw_city_visuals(draw_frame, rois, centers, cw_info_count=cw_count, at_info=at_info, tl_color=tl_color)

        # finalize
        info["steering_angle"] = steering_angle
        self.last_steering_angle = steering_angle

        return steering_angle, draw_frame, info

    # ---------------------- ROI helpers ----------------------
    def _get_rois(self, frame_shape: Tuple[int, int, int]) -> Dict[str, Tuple[int, int, int, int]]:
        height, width = frame_shape[:2]

        def _rect(top_k, bottom_k, left_k, right_k):
            top = int(height * self.config[top_k])
            bottom = int(height * self.config[bottom_k])
            left = int(width * self.config[left_k])
            right = int(width * self.config[right_k])
            # clamp
            top = max(0, min(top, height - 1))
            bottom = max(0, min(bottom, height))
            left = max(0, min(left, width - 1))
            right = max(left + 1, min(right, width))
            return (top, bottom, left, right)

        return {
            "rl": _rect('ROI_TOP_RL', 'ROI_BOTTOM_RL', 'ROI_LEFT_RL', 'ROI_RIGHT_RL'),
            "ll": _rect('ROI_TOP_LL', 'ROI_BOTTOM_LL', 'ROI_LEFT_LL', 'ROI_RIGHT_LL'),
            "cw": _rect('ROI_TOP_CW', 'ROI_BOTTOM_CW', 'ROI_LEFT_CW', 'ROI_RIGHT_CW'),
            "at": _rect('ROI_TOP_AT', 'ROI_BOTTOM_AT', 'ROI_LEFT_AT', 'ROI_RIGHT_AT'),
            "tl": _rect('ROI_TOP_TL', 'ROI_BOTTOM_TL', 'ROI_LEFT_TL', 'ROI_RIGHT_TL')
        }

    # ---------------------- Detection building blocks ----------------------
    def _detect_lanes_only(self, frame: np.ndarray, rois: Dict[str, Tuple[int, int, int, int]]):
        # Process right lane ROI
        top_rl, bottom_rl, left_rl, right_rl = rois['rl']
        roi_frame_rl = frame[top_rl:bottom_rl, left_rl:right_rl]
        center_right_x, right_lines_info, longest_r = self._process_lane_roi(roi_frame_rl, left_rl, right_rl, is_right=True)

        # Process left lane ROI
        top_ll, bottom_ll, left_ll, right_ll = rois['ll']
        roi_frame_ll = frame[top_ll:bottom_ll, left_ll:right_ll]
        center_left_x, left_lines_info, longest_l = self._process_lane_roi(roi_frame_ll, left_ll, right_ll, is_right=False)

        # compute steering
        frame_center = frame.shape[1] // 2
        line_center_x = (center_right_x + center_left_x) / 2
        error = line_center_x - frame_center
        if len(right_lines_info) == 0 and len(left_lines_info) == 0:
            steering_angle = 145
        else:
            steering_angle = current_angle + 0.4 * error
            steering_angle = max(55, min(145, steering_angle))

        longest = max(longest_r, longest_l)
        return steering_angle, (center_right_x, center_left_x), longest

    def _detect_city(self, frame: np.ndarray, rois: Dict[str, Tuple[int, int, int, int]], draw_frame: np.ndarray):
        # lanes
        top_rl, bottom_rl, left_rl, right_rl = rois['rl']
        roi_frame_rl = frame[top_rl:bottom_rl, left_rl:right_rl]
        center_right_x, right_lines_info, longest_r = self._process_lane_roi(roi_frame_rl, left_rl, right_rl, is_right=True)

        top_ll, bottom_ll, left_ll, right_ll = rois['ll']
        roi_frame_ll = frame[top_ll:bottom_ll, left_ll:right_ll]
        center_left_x, left_lines_info, longest_l = self._process_lane_roi(roi_frame_ll, left_ll, right_ll, is_right=False)

        # crosswalk
        top_cw, bottom_cw, left_cw, right_cw = rois['cw']
        roi_frame_cw = frame[top_cw:bottom_cw, left_cw:right_cw]
        cw_lines_info = self._process_crosswalk_roi(roi_frame_cw)

        # apriltag (pass draw ROI slice if drawing enabled)
        top_at, bottom_at, left_at, right_at = rois['at']
        roi_frame_at = frame[top_at:bottom_at, left_at:right_at]
        draw_roi_at = None
        if self.debug and draw_frame is not None:
            draw_roi_at = draw_frame[top_at:bottom_at, left_at:right_at]
        at_tags_info = self._process_apriltag_roi(roi_frame_at, left_at, top_at, draw_roi_at)

        # traffic light
        top_tl, bottom_tl, left_tl, right_tl = rois['tl']
        roi_frame_tl = frame[top_tl:bottom_tl, left_tl:right_tl]
        tl_color = self._process_traffic_light_roi(roi_frame_tl)

        # steering
        frame_center = frame.shape[1] // 2
        line_center_x = (center_right_x + center_left_x) / 2
        error = line_center_x - frame_center
        if len(right_lines_info) == 0 and len(left_lines_info) == 0:
            steering_angle = 145
        else:
            steering_angle = current_angle + 0.4 * error
            steering_angle = max(55, min(145, steering_angle))

        longest = max(longest_r, longest_l)
        return steering_angle, (center_right_x, center_left_x), len(cw_lines_info), at_tags_info, tl_color, longest

    def _process_lane_roi(self, roi_frame: np.ndarray, left_offset: int, right_offset: int, is_right: bool):
        # assume roi_frame is BGR
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, self.config['CANNY_LOW'], self.config['CANNY_HIGH'])
        edge_dilated = cv2.dilate(edge, np.ones((3, 3), np.uint8), iterations=1)
        lines = cv2.HoughLinesP(edge_dilated, 1, np.pi / 180, self.config['HOUGH_THRESHOLD'],
                                minLineLength=self.config['MIN_LINE_LENGTH'], maxLineGap=self.config['MAX_LINE_GAP'])

        height = roi_frame.shape[0]
        center_x = (left_offset + right_offset) // 2
        lines_info: List[tuple] = []
        longest = 0
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x2 == x1:
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    if abs(slope) > 0.3:
                        length = math.hypot(x2 - x1, y2 - y1)
                        # compute x at bottom of ROI
                        denom = slope if abs(slope) > 1e-5 else 1.0
                        x_bottom = x1 + (height - y1) / denom
                        x_bottom = max(0, min(right_offset - left_offset - 1, x_bottom))
                        x_bottom_full = x_bottom + left_offset
                        mid_line_x = (left_offset + right_offset) // 2
                        if is_right and x_bottom_full >= mid_line_x:
                            lines_info.append((x1, y1, x2, y2, x_bottom_full, length))
                        elif not is_right and x_bottom_full <= mid_line_x:
                            lines_info.append((x1, y1, x2, y2, x_bottom_full, length))
                        longest = max(longest, int(length))
        if lines_info:
            center_x = sum(l[4] for l in lines_info) / len(lines_info)
        return center_x, lines_info, longest

    def _process_crosswalk_roi(self, roi_frame: np.ndarray) -> List[tuple]:
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, self.config['CANNY_LOW'], self.config['CANNY_HIGH'])
        lines = cv2.HoughLinesP(edge, 1, np.pi / 180, self.config['HOUGH_THRESHOLD'],
                                minLineLength=20, maxLineGap=self.config['MAX_LINE_GAP'])
        cw_lines = []
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x2 == x1:
                        continue
                    slope = (y2 - y1) / (x2 - x1)
                    if abs(slope) > 2:
                        length = math.hypot(x2 - x1, y2 - y1)
                        if length > 7:
                            cw_lines.append((x1, y1, x2, y2, length))
        return cw_lines

    def _process_apriltag_roi(self, roi_frame: np.ndarray, left_offset: int, top_offset: int, draw_roi: np.ndarray = None) -> List[tuple]:
        """Detect AprilTags (ArUco). If draw_roi provided, draw markers there (ROI coordinates)."""
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        tags = []
        if ids is not None and len(ids) > 0:
            # draw markers only on draw_roi (which is a view into the full draw_frame)
            if draw_roi is not None:
                try:
                    aruco.drawDetectedMarkers(draw_roi, corners, ids)
                except Exception:
                    # some OpenCV builds expect slightly different API — ignore draw failures
                    pass

            # take first id as LAST_TAG (preserve existing behavior)
            try:
                global LAST_TAG
                LAST_TAG = int(ids[0][0])
            except Exception:
                pass

            for i, corner in enumerate(corners):
                pts = corner[0].astype(int)  # corner pts are ROI-local
                # compute area (shoelace)
                area = 0.5 * abs(
                    sum(pts[j][0] * pts[(j + 1) % 4][1] for j in range(4)) -
                    sum(pts[j][1] * pts[(j + 1) % 4][0] for j in range(4))
                )
                tag_id = int(ids[i][0])
                tags.append((tag_id, pts.tolist(), area))
                # if tag very large, stop robot (safety rule)
                if area > 2000:
                    try:
                        send_command(b'STOP\r\n')
                    except Exception:
                        logger.exception("Failed to send STOP for big AprilTag")
        return tags

    def _process_traffic_light_roi(self, roi_frame: np.ndarray) -> str:
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        # tuning ranges (BGR->HSV) — kept from original but adjusted as arrays
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([95, 255, 255])
        yellow_lower = np.array([15, 80, 80])
        yellow_upper = np.array([35, 255, 255])

        mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
        mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        yellow_pixels = cv2.countNonZero(mask_yellow)

        if red_pixels > 250:
            return "Red"
        if green_pixels > 6:
            return "Green"
        if yellow_pixels > max(red_pixels, green_pixels, 3):
            return "Yellow"
        return "None"

    # ---------------------- Drawing helpers ----------------------
    def _draw_lane_visuals(self, draw_frame: np.ndarray, rois: Dict[str, Tuple[int, int, int, int]], centers: tuple, lane_type: str = "both"):
        """Draw lane ROIs and centers onto draw_frame. draw_frame must be not None."""
        if draw_frame is None:
            return
        left_rl = rois['rl'][2]
        right_rl = rois['rl'][3]
        top_rl = rois['rl'][0]
        bottom_rl = rois['rl'][1]
        left_ll = rois['ll'][2]
        right_ll = rois['ll'][3]
        top_ll = rois['ll'][0]
        bottom_ll = rois['ll'][1]

        center_right_x, center_left_x = centers
        # Right ROI (yellow)
        cv2.rectangle(draw_frame, (left_rl, top_rl), (right_rl, bottom_rl), (0, 255, 255), 2)
        cv2.putText(draw_frame, "RL", (left_rl + 5, top_rl + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # Left ROI (cyan)
        cv2.rectangle(draw_frame, (left_ll, top_ll), (right_ll, bottom_ll), (255, 255, 0), 2)
        cv2.putText(draw_frame, "LL", (left_ll + 5, top_ll + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # Draw lane center lines
        cv2.line(draw_frame, (int(center_right_x), top_rl), (int(center_right_x), bottom_rl), (255, 0, 0), 2)
        cv2.line(draw_frame, (int(center_left_x), top_ll), (int(center_left_x), bottom_ll), (0, 0, 255), 2)

    def _draw_city_visuals(self, draw_frame: np.ndarray, rois: Dict[str, Tuple[int, int, int, int]], centers: tuple, cw_info_count: int, at_info: List[tuple], tl_color: str):
        """Draw city overlays (lanes, crosswalk, apriltags, traffic light) on draw_frame."""
        if draw_frame is None:
            return
        # draw lane + cw + at + tl overlays
        self._draw_lane_visuals(draw_frame, rois, centers)

        left_cw, top_cw, right_cw, bottom_cw = rois['cw'][2], rois['cw'][0], rois['cw'][3], rois['cw'][1]
        left_at, top_at, right_at, bottom_at = rois['at'][2], rois['at'][0], rois['at'][3], rois['at'][1]
        left_tl, top_tl, right_tl, bottom_tl = rois['tl'][2], rois['tl'][0], rois['tl'][3], rois['tl'][1]

        # Crosswalk ROI (green)
        cv2.rectangle(draw_frame, (left_cw, top_cw), (right_cw, bottom_cw), (0, 255, 0), 2)
        cv2.putText(draw_frame, "CW", (left_cw + 5, top_cw + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # AprilTag ROI (green)
        cv2.rectangle(draw_frame, (left_at, top_at), (right_at, bottom_at), (0, 255, 0), 2)
        cv2.putText(draw_frame, "AT", (left_at + 5, top_at + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # Traffic light ROI (magenta)
        cv2.rectangle(draw_frame, (left_tl, top_tl), (right_tl, bottom_tl), (255, 0, 255), 2)
        cv2.putText(draw_frame, "TL", (left_tl + 5, top_tl + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # Draw AprilTag boxes and ids (at_info contains ROI-local corners)
        for tag_id, corners, area in at_info:
            pts = [(int(c[0]) + left_at, int(c[1]) + top_at) for c in corners]
            for i in range(4):
                cv2.line(draw_frame, pts[i], pts[(i + 1) % 4], (0, 255, 0), 2)
            cx = int(sum(p[0] for p in pts) / 4)
            cy = int(sum(p[1] for p in pts) / 4)
            cv2.putText(draw_frame, f"ID: {tag_id}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw traffic light color
        cv2.putText(draw_frame, f"Color: {tl_color}", (left_tl + 5, top_tl + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # stats
        cv2.putText(draw_frame, f"CW Lines: {cw_info_count}", (left_cw + 5, top_cw + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(draw_frame, f"AT Tags: {len(at_info)}", (left_at + 5, top_at + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
