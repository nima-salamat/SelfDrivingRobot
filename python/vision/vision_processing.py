import math
from config import (
    RL_TOP_ROI, RL_BOTTOM_ROI, RL_RIGHT_ROI, RL_LEFT_ROI,
    LL_TOP_ROI, LL_BOTTOM_ROI, LL_RIGHT_ROI, LL_LEFT_ROI,
    CW_TOP_ROI, CW_BOTTOM_ROI, CW_RIGHT_ROI, CW_LEFT_ROI,
    LOW_KP, HIGH_KP, MAX_SERVO_ANGLE, MIN_SERVO_ANGLE
)
import config
import cv2
import numpy as np

class VisionProcessor:
    def __init__(self):
        self.last_steering = 90

    def _largest_mid_x(self, lines):
        if lines is None:
            return None
        max_length = 0
        x_mid = None
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1 + 1e-9)
            if abs(slope) > 0.1:
                length = math.hypot(x2 - x1, y2 - y1)
                if length > max_length:
                    max_length = length
                    x_mid = (x1 + x2) / 2.0
        return x_mid

    def detect(self, frame):
        height, width = frame.shape[:2]

        # --- ROI pixel bounds ---
        rl_top, rl_bottom = int(RL_TOP_ROI * height), int(RL_BOTTOM_ROI * height)
        rl_left, rl_right = int(RL_LEFT_ROI * width), int(RL_RIGHT_ROI * width)
        ll_top, ll_bottom = int(LL_TOP_ROI * height), int(LL_BOTTOM_ROI * height)
        ll_left, ll_right = int(LL_LEFT_ROI * width), int(LL_RIGHT_ROI * width)

        cw_top, cw_bottom = int(CW_TOP_ROI * height), int(CW_BOTTOM_ROI * height)
        cw_left, cw_right = int(CW_LEFT_ROI * width), int(CW_RIGHT_ROI * width)

        # --- Crop ROIs (ROI-local coordinate space) ---
        rl_frame = frame[rl_top:rl_bottom, rl_left:rl_right].copy()
        ll_frame = frame[ll_top:ll_bottom, ll_left:ll_right].copy()
        cw_frame = frame[cw_top:cw_bottom, cw_left:cw_right].copy()

        rl_frame = rl_frame if (rl_frame is not None and rl_frame.size != 0) else None
        ll_frame = ll_frame if (ll_frame is not None and ll_frame.size != 0) else None
        cw_frame = cw_frame if (cw_frame is not None and cw_frame.size != 0) else None

        # --- Process ROI: gray -> blur -> edges -> HoughLinesP ---
        def process_roi(roi):
            if roi is None:
                return None, None, None
            roi_copy = roi.copy()
            gray = cv2.cvtColor(roi_copy, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            _, gray = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
            edges = cv2.Canny(gray, 50, 100)
            lines = cv2.HoughLinesP(edges, 3, np.pi / 180, 50, minLineLength=20, maxLineGap=5)
            return roi_copy, edges, lines

        rl_draw, rl_edge, rl_lines = process_roi(rl_frame)
        ll_draw, ll_edge, ll_lines = process_roi(ll_frame)

        # -------------------------
        # CROSSWALK DETECTION USING LSD
        # -------------------------
        crosswalk = False
        cw_debug = None

        if cw_frame is not None:
            # Preprocess cw ROI
            cw_gray = cv2.cvtColor(cw_frame, cv2.COLOR_BGR2GRAY)

            # Use a slightly adaptive strategy: try high fixed threshold first; if too dark, fallback to adaptive
            _, cw_bin = cv2.threshold(cw_gray, 220, 255, cv2.THRESH_BINARY)
            if np.count_nonzero(cw_bin) < 10:  # fallback if threshold produces nearly-empty result
                cw_bin = cv2.adaptiveThreshold(cw_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                               cv2.THRESH_BINARY, 11, 2)

            # Morphological clean
            kernel = np.ones((5, 5), np.uint8)
            cw_bin = cv2.morphologyEx(cw_bin, cv2.MORPH_CLOSE, kernel)
            cw_bin = cv2.morphologyEx(cw_bin, cv2.MORPH_OPEN, kernel)

            # Create LSD and detect lines on the binary (or grayscale) image
            try:
                lsd = cv2.createLineSegmentDetector(0)
                # LSD expects a gray image (float or 8u); pass cw_bin to emphasize stripes
                lsd_lines = lsd.detect(cw_bin)[0]  # returns tuple (lines, width, prec, nfa)
            except Exception:
                # If createLineSegmentDetector not available or fails, fallback to Hough
                lsd_lines = None

            cw_draw = cv2.cvtColor(cw_bin, cv2.COLOR_GRAY2BGR)

            # collect useful lines
            detected_lines = []
            if lsd_lines is not None:
                # lsd_lines can have shapes (N,1,4) or (N,4)
                for l in lsd_lines:
                    coords = l.reshape(-1)  # flatten
                    if coords.size >= 4:
                        x1, y1, x2, y2 = coords[:4]
                        length = math.hypot(x2 - x1, y2 - y1)
                        if length < 15:  # ignore very short segments
                            continue
                        detected_lines.append((float(x1), float(y1), float(x2), float(y2), length))

            # If no LSD lines found, fallback to HoughLinesP on cw_bin edges
            if not detected_lines:
                edges = cv2.Canny(cw_bin, 50, 150)
                lines_p = cv2.HoughLinesP(edges, 3, np.pi/180, 30, minLineLength=20, maxLineGap=10)
                if lines_p is not None:
                    for line in lines_p:
                        x1,y1,x2,y2 = line[0]
                        length = math.hypot(x2-x1, y2-y1)
                        if length < 15:
                            continue
                        detected_lines.append((float(x1), float(y1), float(x2), float(y2), length))

            # analyze orientations
            angles = []
            for (x1,y1,x2,y2,le) in detected_lines:
                angle = math.degrees(math.atan2((y2-y1), (x2-x1+1e-9)))
                # normalize to [-90,90]
                if angle <= -90:
                    angle += 180
                if angle > 90:
                    angle -= 180
                angles.append(angle)

            # count horizontal vs vertical-like lines
            angle_tol = 20  # degrees tolerance for grouping
            horizontal_lines = []
            vertical_lines = []
            for i, (x1,y1,x2,y2,le) in enumerate(detected_lines):
                a = angles[i]
                if abs(a) <= angle_tol:  # near 0 deg -> horizontal
                    horizontal_lines.append((x1,y1,x2,y2,le,a))
                if abs(abs(a) - 90) <= angle_tol:  # near +-90 deg -> vertical
                    vertical_lines.append((x1,y1,x2,y2,le,a))

            # choose dominant orientation (crosswalk stripes commonly horizontal across image)
            dominant = 'horizontal' if len(horizontal_lines) >= len(vertical_lines) else 'vertical'
            candidate_lines = horizontal_lines if dominant == 'horizontal' else vertical_lines

            # draw all detected lines faintly
            for (x1,y1,x2,y2,le) in detected_lines:
                cv2.line(cw_draw, (int(x1), int(y1)), (int(x2), int(y2)), (60,60,60), 1)

            # highlight candidate lines and collect midpoints
            midpoints = []
            for (x1,y1,x2,y2,le,a) in candidate_lines:
                cv2.line(cw_draw, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,255), 2)
                mx = (x1 + x2) / 2.0
                my = (y1 + y2) / 2.0
                midpoints.append((mx,my))

            # simple heuristic: if we have >= min_stripes candidate parallel segments -> crosswalk
            min_stripes = 3
            if len(candidate_lines) >= min_stripes:
                # additional spacing check: ensure midpoints are not all clustered
                if dominant == 'horizontal':
                    ys = sorted([int(p[1]) for p in midpoints])
                    # require at least 3 distinct y positions with minimum spacing (in pixels)
                    distinct = 1
                    for i in range(1, len(ys)):
                        if abs(ys[i] - ys[i-1]) > max(5, int((cw_bottom-cw_top)*0.03)):
                            distinct += 1
                    if distinct >= 3:
                        crosswalk = True
                else:  # vertical
                    xs = sorted([int(p[0]) for p in midpoints])
                    distinct = 1
                    for i in range(1, len(xs)):
                        if abs(xs[i] - xs[i-1]) > max(5, int((cw_right-cw_left)*0.03)):
                            distinct += 1
                    if distinct >= 3:
                        crosswalk = True

            cw_debug = cw_draw

        # -------------------------
        # LANE MIDPOINT (unchanged)
        # -------------------------
        rl_x_mid = self._largest_mid_x(rl_lines)
        ll_x_mid = self._largest_mid_x(ll_lines)

        rl_x_mid_full = (rl_left + rl_x_mid) if rl_x_mid is not None else None
        ll_x_mid_full = (ll_left + ll_x_mid) if ll_x_mid is not None else None

        if (rl_x_mid_full is not None) and (ll_x_mid_full is not None):
            lane_type = "both"
        elif (rl_x_mid_full is None) and (ll_x_mid_full is not None):
            lane_type = "only_left"
        elif (rl_x_mid_full is not None) and (ll_x_mid_full is None):
            lane_type = "only_right"
        else:
            lane_type = "none"

        frame_center = (width * (RL_RIGHT_ROI + LL_LEFT_ROI) / 2) + 5

        rl_roi_center = (rl_left + rl_right) / 2.0
        ll_roi_center = (ll_left + ll_right) / 2.0

        if rl_x_mid_full is None and ll_x_mid_full is not None:
            rl_x_mid_full = rl_roi_center
        if ll_x_mid_full is None and rl_x_mid_full is not None:
            ll_x_mid_full = ll_roi_center

        if lane_type in ("both", "only_right", "only_left"):
            lane_center = (rl_x_mid_full + ll_x_mid_full) / 2.0
        else:
            lane_center = frame_center

        error = lane_center - frame_center
        kp = LOW_KP if abs(error) < 25 else HIGH_KP
        steering_angle = 90.0 + kp * error
        if lane_type == "none":
            steering_angle = 150
        steering_angle = int(max(MIN_SERVO_ANGLE, min(MAX_SERVO_ANGLE, steering_angle)))

        # -------------------------
        # DEBUG DRAWING
        # -------------------------
        debug = {"rl_draw": None, "ll_draw": None, "combined": None, "crosswalk_draw": cw_debug}

        if config.DEBUG:
            vis = frame.copy()

            # ROI boxes
            cv2.rectangle(vis, (rl_left, rl_top), (rl_right, rl_bottom), (255, 0, 0), 1)
            cv2.rectangle(vis, (ll_left, ll_top), (ll_right, ll_bottom), (0, 255, 0), 1)
            cv2.rectangle(vis, (cw_left, cw_top), (cw_right, cw_bottom), (0, 255, 255), 1)

            # draw Hough lines from RL ROI (into global image)
            if rl_lines is not None:
                for line in rl_lines:
                    x1, y1, x2, y2 = line[0]
                    # draw on rl ROI copy if available
                    if rl_draw is not None:
                        cv2.line(rl_draw, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 1)
                    # draw on global vis (with offset)
                    cv2.line(vis, (rl_left + int(x1), rl_top + int(y1)), (rl_left + int(x2), rl_top + int(y2)), (0,255,0), 2)
                if rl_x_mid_full is not None:
                    cv2.circle(vis, (int(rl_x_mid_full), int((rl_top + rl_bottom)/2)), 4, (0,255,0), -1)

            # draw Hough lines from LL ROI
            if ll_lines is not None:
                for line in ll_lines:
                    x1, y1, x2, y2 = line[0]
                    if ll_draw is not None:
                        cv2.line(ll_draw, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 1)
                    cv2.line(vis, (ll_left + int(x1), ll_top + int(y1)), (ll_left + int(x2), ll_top + int(y2)), (0,255,0), 2)
                if ll_x_mid_full is not None:
                    cv2.circle(vis, (int(ll_x_mid_full), int((ll_top + ll_bottom)/2)), 4, (0,255,0), -1)

            # show lane center / frame center
            cv2.line(vis, (int(frame_center), 0), (int(frame_center), height), (0,0,255), 1)
            cv2.line(vis, (int(lane_center), 0), (int(lane_center), height), (255,0,255), 1)

            # crosswalk text and paste cw_debug into the cw ROI for inspection
            cv2.putText(vis, f"crosswalk:{crosswalk}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

            if cw_debug is not None:
                try:
                    vis[cw_top:cw_bottom, cw_left:cw_right] = cv2.resize(cw_debug, (cw_right - cw_left, cw_bottom - cw_top))
                except Exception:
                    # if paste fails, ignore (still keep vis)
                    pass

            debug["rl_draw"] = rl_draw
            debug["ll_draw"] = ll_draw
            debug["combined"] = vis

        return {
            "steering_angle": steering_angle,
            "error": error,
            "lane_type": lane_type,
            "crosswalk": crosswalk,
            "crosswalk_debug": cw_debug,
            "debug": debug
        }
