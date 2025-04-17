
import time 
from config import MODE, ANGLE_CENTER, ANGLE_SHARP_RIGHT, ANGLE_SHARP_LEFT
class IntersectionNavigator:
    def __init__(self, ser):
        self.ser = ser
        self.forward_speed = 150
        self.turn_speed = 150
        self.angle_right = 160
        self.angle_left = 0
        self.angle_center = 100
    
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
        time.sleep(0.5)
        self.go_left
        time.sleep(0.75)
        self.go_forward
        time.sleep(0.5)
        self.go_left
        time.sleep(0.5)    
        self.go_forward
        time.sleep(0.5)
        self.stop
        time.sleep(1)

    def turn_right(self):
        self.go_forward
        time.sleep(0.3)
        self.go_right
        time.sleep(2)
        self.go_forward
        time.sleep(0.1)
        self.stop
        time.sleep(1)

    def forward(self):
        self.go_forward
        time.sleep(3)
    
    def navigate_by_tag(self, label):
        if label == "turn left":
            self.turn_left()
        elif label == "turn right":
            self.turn_right()
        elif label == "go straight":
            self.forward() 
    