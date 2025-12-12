import math
from config_city import (
    CW_TOP_ROI, CW_BOTTOM_ROI, CW_RIGHT_ROI, CW_LEFT_ROI,
)
import cv2
import numpy as np


class CrosswalkProcess:
    def __init__(self, manager_dict):
        self.manager_dict = manager_dict
        
    def detect(self, frame):
        height, width = frame.shape[:2]

        cw_top, cw_bottom = int(CW_TOP_ROI * height), int(CW_BOTTOM_ROI * height)
        cw_left, cw_right = int(CW_LEFT_ROI * width), int(CW_RIGHT_ROI * width)

        cw_frame = frame[cw_top:cw_bottom, cw_left:cw_right]

       
        cw_frame = cw_frame if (cw_frame is not None and cw_frame.size != 0) else None


        crosswalk = False

        if cw_frame is not None:
            gray = cv2.cvtColor(cw_frame, cv2.COLOR_BGR2GRAY)
            _, gray = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
            edges = cv2.Canny(gray, 100, 150)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20,
                        minLineLength=5, maxLineGap=5)

            vertical = 0
            horizontal = 0
            if lines is not None:
                for line in lines:
                    x0, y0, x1, y1 = line[0]
                    slope = (y1 - y0) / (x1 - x0 + 1e-6)
                    angle = abs(np.arctan(slope) * 180 / np.pi)

                    if angle < 30:
                        horizontal += 1
                    elif angle > 60:
                        vertical += 1
            
            if vertical > 3 and horizontal > 3:
                crosswalk = True
        return crosswalk

    def runner(self):
        while True:
            frame = self.manager_dict["frame"]
            if frame is None:
                    continue
            self.manager_dict['crosswalk'] = self.detect(frame)