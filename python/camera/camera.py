
import cv2
import numpy as np
from typing import Optional
import logging

logger = logging.getLogger(__name__)

try:
    from picamera2 import Picamera2
except ImportError:
    logger.error("picamera library is not installed")
    Picamera2 = None
from config import F_Width, F_Height
import time 


class Camera:
    def __init__(self, width: int = F_Width, height: int = F_Height) -> None:
        self.width = width
        self.height = height
        self.cap = None

    def capture_frame(self, frame, flip=False) ->Optional[np.ndarray]:
        # Ensure frame is resized to configured size
        frame = cv2.resize(frame, (self.width, self.height))
        
        if flip:
            # Rotate frame 180 degree
            frame = cv2.flip(frame, -1)
            
        return frame
    
    
class USBCamera(Camera):
    def __init__(self, width: int = F_Width, height: int = F_Height, init: bool=True) -> None:
        super().__init__(width, height)
        self.init = init
        if init:
            self._initialize_camera()
    
    def _initialize_camera(self, index=0) -> None:
        logger.info("usb-camera is initializing . . . ")
        camera = cv2.VideoCapture(index)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap = camera
        logger.info("usb-camera is captured")

    def capture_frame(self) -> Optional[np.ndarray]:
        if self.init and self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                return
            return super().capture_frame(frame, flip=False)
        
        logger.warning("usb-camera self.cap is None or not initialized")


class PICamera(Camera):
    def __init__(self, width: int = F_Width, height: int = F_Height) -> None:
        super().__init__(width, height)
        self._initialize_camera()
    
    def _initialize_camera(self) -> None:
        
        logger.info("pi-camera is initializing . . . ")
        if Picamera2 is None:
            logger.error("Picamera2 class from picamera2 lib didnt import (picamera2 does not exists)")
            return
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.set_controls({
            "AeEnable": False,
            "AwbEnable": False,
            "AnalogueGain": 1.0,
            "ExposureTime": 50000,
            "Brightness": 0.0,
            "Contrast": 1.0,
            "Saturation": 1.0,
            "Sharpness": 0.0,
            "NoiseReductionMode": 0,
            "FrameDurationLimits": (50000, 50000),
        })
        picam2.start()
        self.cap = picam2
        time.sleep(1)
        logger.info("pi-camera is initialized")
        
    def capture_frame(self) -> Optional[np.ndarray]:
        if self.cap is not None:
            frame = self.cap.capture_array()
            return super().capture_frame(frame, flip=True)
        logger.warning("pi-camera self.cap is None")
