# import cv2
# from picamera2 import Picamera2
# import time
# from config import (TARGET_FPS,
#                     ROI_Y_START_FRAC,
#                     ROI_X_START_FRAC,
#                     ROI_X_END_FRAC,
#                     LANE_WIDTH_FRAC,
#                     CAMERA_WIDTH,
#                     CAMERA_HEIGHT
#                     )


# # Initialize camera capture
# def initialize_camera(index=0, width=480, height=240):
    
#     # cap = cv2.VideoCapture(CAMERA_INDEX)
#     cap = cv2.VideoCapture(index)
#     if not cap.isOpened():
#         print("Error: Could not open camera.")
#         exit()

#     cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
#     # Get video resolution


#     ret, frame = cap.read()
#     if ret:
#         height, width = frame.shape[:2]
#         print(f"Camera resolution: {width}x{height}")
#     else:
#         print("Error reading from camera.")
#         exit()
        
#     return cap, width, height


# class Camera:
#     def __init__(self, index=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
#         result = initialize_camera(index, width, height)
#         self.cap, self.width, self.height = result    

        
# class CameraBottom(Camera):
#     def __init__(self, index):
#         super().__init__(index)
#         self.initialize_fixed_roi()
        
#     def initialize_fixed_roi(self):
#         self.crosswalk_roi = (
#             self.width // 4,
#             (3 * self.height) // 5,
#             self.width // 2,
#             self.height // 5,
#         )
#         self.ROI_Y_START = int(self.height * ROI_Y_START_FRAC)
#         self.ROI_Y_END = int(self.height * 1.0)
#         self.ROI_X_START = int(self.width * ROI_X_START_FRAC)
#         self.ROI_X_END = int(self.width * ROI_X_END_FRAC)
#         self.ROI_WIDTH = self.ROI_X_END - self.ROI_X_START
#         self.LANE_WIDTH = int(self.width * LANE_WIDTH_FRAC)
        
#     def read(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             return ret, frame
#         roi = frame[self.ROI_Y_START:self.ROI_Y_END, self.ROI_X_START:self.ROI_X_END]
#         return ret, roi
    
# class CameraTop:
#     class cap_:
#         def __init__(self, picam):
#             self.picam = picam
#             self.is_open = False
            
#         def read(self):
#             try:
#                 frame = self.picam.capture_array()
#                 frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#                 return True, frame_bgr
#             except Exception as e:
#                 print(f"Error capturing frame: {e}")
#                 return False, None
            
#         def release(self):
#             if self.is_open:
#                 self.picam.stop()
#                 self.is_open = False
#     def __init__(self):
#        # Initialize the camera
#         picam2 = Picamera2()

#         # Configure camera (preview configuration is good for live video)
#         config = picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
#         picam2.configure(config)

#         # Start the camera
#         picam2.start() 
#         self.picam = picam2
#         self.cap = self.cap_(self.picam)
#         self.cap.is_open = True
#         self.width = CAMERA_WIDTH
#         self.height = CAMERA_HEIGHT
        
import cv2
from picamera2 import Picamera2
import time
from typing import Tuple, Optional
from config import (
    TARGET_FPS,
    ROI_Y_START_FRAC,
    ROI_X_START_FRAC,
    ROI_X_END_FRAC,
    LANE_WIDTH_FRAC,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
)


class CameraBase:
    def __init__(self):
        self._width = CAMERA_WIDTH
        self._height = CAMERA_HEIGHT
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def resolution(self) -> Tuple[int, int]:
        return (self._width, self._height)

    def read(self):
        raise NotImplementedError

    def release(self):
        raise NotImplementedError

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


class USBCamera(CameraBase):
    def __init__(
        self, index=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, max_retries=3
    ):
        super().__init__()
        self._index = index
        self._max_retries = max_retries

        for attempt in range(max_retries):
            try:
                self._cap = cv2.VideoCapture(index)
                if not self._cap.isOpened():
                    continue

                self._cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

                # Verify actual resolution
                ret, frame = self._cap.read()
                if ret:
                    self._height, self._width = frame.shape[:2]
                    self._is_open = True
                    print(f"USB Camera initialized at {self._width}x{self._height}")
                    break
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {str(e)}")
                time.sleep(1)

        if not self._is_open:
            raise RuntimeError(
                f"Failed to initialize USB camera after {max_retries} attempts"
            )

    def read(self):
        if not self._is_open:
            return False, None

        ret, frame = self._cap.read()
        if not ret:
            return False, None

        return True, frame

    def release(self):
        if hasattr(self, "_cap") and self._cap.isOpened():
            self._cap.release()
        self._is_open = False


class PiCamera(CameraBase):
    def __init__(self, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        super().__init__()
        self._picam = Picamera2()

        try:
            config = self._picam.create_preview_configuration(
                main={"size": (width, height), "format": "RGB888"}
            )
            self._picam.configure(config)
            self._picam.start()
            self._is_open = True
            self._width = width
            self._height = height
            print(f"PiCamera initialized at {width}x{height}")
        except Exception as e:
            self._is_open = False
            raise RuntimeError(f"PiCamera initialization failed: {str(e)}")

    def read(self) :
        if not self._is_open:
            return False, None

        try:
            frame = self._picam.capture_array()
            return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"Capture error: {str(e)}")
            return False, None

    def release(self):
        if hasattr(self, "_picam"):
            self._picam.stop()
        self._is_open = False