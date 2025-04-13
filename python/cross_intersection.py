
import time 
from config import MODE, ANGLE_CENTER, ANGLE_SHARP_RIGHT, ANGLE_SHARP_LEFT
class IntersectionNavigator:
    def __init__(self, ser):
        self.ser = ser
        self.forward_speed = MODE.default.forward
        self.turn_speed = MODE.default.turn
        self.angle_right = ANGLE_SHARP_RIGHT
        self.angle_left = ANGLE_SHARP_LEFT
        self.angle_center = ANGLE_CENTER
    
    @property
    def go_forward(self):
        self.ser.send(f"command {self.angle_center} {self.forward_speed}")
        
    @property
    def go_right(self):
        self.ser.send(f"command {self.angle_right} {self.turn_speed}")
    
    @property
    def go_left(self):
        self.ser.send(f"command {self.angle_left} {self.turn_speed}")

    @property
    def stop(self):
        self.ser.send(f"command {self.angle_center} 0")

    def turn_left(self):
        self.go_forward
        time.time(0.4)
        self.go_left
        time.time(0.6)    
        self.go_forward
        time.time(0.2)
        self.stop
        
    def turn_right(self):
        self.go_forward
        time.time(0.3)
        self.go_right
        time.time(0.4)
        self.go_forward
        time.time(0.3)
        self.stop
        
    def forward(self):
        self.go_forward
        time.time(1)
    
    def navigate_by_tag(self, label):
        if label == "turn left":
            self.turn_left()
        elif label == "turn right":
            self.turn_right()
        elif label == "go straight":
            self.forward() 
    