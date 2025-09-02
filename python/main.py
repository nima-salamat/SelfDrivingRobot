from detectors.lane.lane_detection import LaneDetector
from detectors.trafficlight.trafficlight_detection import TrafficLightDetector
from detectors.ultrasonic.ultrasonic import Ultrasonic
from detectors.apriltag.apriltag_detection import ApriltagDetector
from crossroads.cross_intersection import IntersectionNavigator
from camera  import PiCamera
from serial.connector import connect as serial_connector, serial
import time
import threading

LABEL = None

def check_apriltag():
    global LABEL
    LABEL = None
    for _ in range(5):
        if (label:=ApriltagDetector().detect()) and LABEL is None:
            LABEL = label
        
class CoreRobot:  

    def __init__(self):
        self._ser = None
        self.camera = PiCamera()
        self.connect()
        self.lane_detector = LaneDetector(serial=self.ser, camera=self.camera)
        self.intersection_nav = IntersectionNavigator(self.ser)
    
    def detect(self):
        _, frame = self.camera.cap.read()
        is_crosswalk, _ = self.lane_detector.detect(frame)
        if is_crosswalk:
            threading.Thread(check_apriltag).start()
            time.sleep(5)
            global LABEL
            self.intersection_nav.navigate_by_tag(LABEL)

    @property
    def ser(self):
        return self._ser
    
    @ser.setter
    def ser(self, value):
        if isinstance(value, serial.Serial):
            self._ser = value
            return 
        raise ValueError("")
        
    @ser.getter
    def ser(self):
        if self._ser is None:
            raise ValueError("ser cant be null")
        return self._ser
    
    def connect(self):
        self.ser = serial_connector()
    
    def disconnect(self):
        self.ser.close()

class MainControl:
    def __init__(self):
        self.robot = CoreRobot()
        self.running = True
    def run(self):
        while self.running:
            self.robot.detect()


if __name__ == "__main__":
    control = MainControl()
    try:
        control.run()
    except:
        control.disconnect()
        

