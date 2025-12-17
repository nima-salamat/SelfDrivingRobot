import logging
import cv2
import sys

logger = logging.getLogger(__name__)

try:
    from kivy.utils import platform
    is_android = platform == 'android'
except ImportError:
    is_android = False

class Camera:
    """
    Cross-platform camera for desktop (OpenCV) and Android (OpenCV via Pydroid).
    Headless: only captures frames, no GUI widget needed.
    """
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.cap = None
        self.camera_initialized = False
        self.setup_camera()

    def setup_camera(self):
        try:
            # OpenCV works both on desktop and Pydroid Android
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                logger.error("Failed to open camera")
                self.camera_initialized = False
                return

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.camera_initialized = True
            logger.info("Camera initialized successfully")
        except Exception as e:
            logger.error(f"Camera setup failed: {e}")
            self.camera_initialized = False

    def capture_frame(self):
        if not self.camera_initialized:
            logger.error("Camera not initialized")
            return None
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                logger.warning("Failed to capture frame")
                return None
            return frame
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None

    def release(self):
        if self.cap:
            self.cap.release()
        self.camera_initialized = False
        logger.info("Camera released")
