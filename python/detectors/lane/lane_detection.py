import cv2 as cv
from config import THRESHOLD_VALUE_SIMPLE_LANE_DETECTION as threshold_value
import re


class SimpleLaneDetection:

    def __init__(self, serial, points=[], width=None, height=None, point_y=None, 
                 camera=None, point_per_pixel=10, margin_left=10, margin_right=10,
                 threshold_y=0.6, min_angle=0, max_angle=180, speed=200):
        
        self.speed = speed
        self.serial = serial
        self.points = points
        self.width = width
        self.height = height
        
        # camera is unnecessary if it is set automatically detect width, height, points, point_y
        self.camera = camera
        
        self.point_per_pixel = point_per_pixel
        self.margin_left =  margin_left
        self.margin_right = margin_right
        self.point_y = point_y
        self.threshold_y = threshold_y
        self.min_angle = min_angle
        self.max_angle = max_angle
        if not self.points:
        
            if self.camera:
                self.width = self.camera.width
                self.height = self.camera.height
                self.point_y = int(self.height * self.threshold_y)
            
            elif self.width and self.height and self.point_y:
                start = 0
                end = self.width - self.margin_left - self.margin_right
                step = self.width // self.point_per_pixel # how many point
                self.points = [
                    (self.point_y, self.margin_left + x)
                    for x in range(start, end, step)
                ]
                
            else: 
                raise
    
    
    def normalize(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        _, thresh = cv.threshold(gray, threshold_value, 1, cv.THRESH_BINARY)
        return thresh
    
    
    def check_pattern(self, command_text):
        pattern = r'^0*1+0*$'
        return bool(re.fullmatch(pattern, command_text))


    def calculate_steering(self, command_text):
        start = command_text.find("1") 
        end = command_text.rfind("1")
        center = (end+start)//2
        angle = int((self.max_angle - self.min_angle) * center / (len(self.points)-1) + self.min_angle)
        self.serial.send(f"command {angle} {self.speed}")
        

    def detect(self, frame):
        thresh = self.normalize(frame)
        command_text = "".join(str(thresh[y, x]) for y, x in self.points)
        if "1" not in command_text:
            # lost line
            self.serial.send(f"command {self.max_angle} {self.speed}")
            return False, command_text
        
        if self.check_pattern(command_text):
            self.calculate_steering(command_text)
            return False, command_text
        
        return True, command_text
        
        
        # return parameters   1. is it cross walk for apriltag detection and handle stop   2. command_text 