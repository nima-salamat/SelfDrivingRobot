import cv2
from picamera2 import Picamera2
import time
from config import (TARGET_FPS,
                    ROI_Y_START_FRAC,
                    ROI_X_START_FRAC,
                    ROI_X_END_FRAC,
                    LANE_WIDTH_FRAC,
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT
                    )


# Initialize camera capture
def initialize_camera(index=0, width=480, height=240):
    
    # cap = cv2.VideoCapture(CAMERA_INDEX)
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Get video resolution


    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        print(f"Camera resolution: {width}x{height}")
    else:
        print("Error reading from camera.")
        exit()
    print(width, height)
    return cap, width, height


class Camera:
    def __init__(self, index=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        result = initialize_camera(index, width, height)
        self.cap, self.width, self.height = result    

        
class UsbCamera(Camera):
    def __init__(self, index):
        super().__init__(index)
        self.initialize_fixed_roi()
        
    def initialize_fixed_roi(self):
        self.crosswalk_roi = (
            self.width // 4,
            (3 * self.height) // 5,
            self.width // 2,
            self.height // 5,
        )
        self.ROI_Y_START = int(self.height * ROI_Y_START_FRAC)
        self.ROI_Y_END = int(self.height * 1.0)
        self.ROI_X_START = int(self.width * ROI_X_START_FRAC)
        self.ROI_X_END = int(self.width * ROI_X_END_FRAC)
        self.ROI_WIDTH = self.ROI_X_END - self.ROI_X_START
        self.LANE_WIDTH = int(self.width * LANE_WIDTH_FRAC)
        
    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return ret, frame
        roi = frame[self.ROI_Y_START:self.ROI_Y_END, self.ROI_X_START:self.ROI_X_END]
        return ret, roi
    
class PiCamera:
    class cap_:
        def __init__(self, picam):
            self.picam = picam
            self.is_open = False
            
        def read(self):
            try:
                frame = self.picam.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                return True, frame_bgr
            except Exception as e:
                print(f"Error capturing frame: {e}")
                return False, None
            
        def release(self):
            if self.is_open:
                self.picam.stop()
                self.is_open = False
    def __init__(self):
       # Initialize the camera
        picam2 = Picamera2()

        # Configure camera (preview configuration is good for live video)
        config = picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
        picam2.configure(config)

        # Start the camera
        picam2.start() 
        self.picam = picam2
        self.cap = self.cap_(self.picam)
        self.cap.is_open = True
        self.width = CAMERA_WIDTH
        self.height = CAMERA_HEIGHT
        
