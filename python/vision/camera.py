import logging
import cv2

try:
    from kivy.utils import platform
    is_android = platform == 'android'
except ImportError:
    is_android = False

logger = logging.getLogger(__name__)

class Camera:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.camera_initialized = False
        self.cap = None
        self.kivy_camera = None
        self.setup_camera()

    def setup_camera(self):
        if is_android:
            try:
                from kivy.uix.camera import Camera as KivyCamera
                self.kivy_camera = KivyCamera(index=0, resolution=(self.width, self.height))
                self.camera_initialized = True
                logger.info("Kivy camera initialized successfully for Android.")
            except Exception as e:
                self.camera_initialized = False
                logger.error(f"Error initializing Kivy camera on Android: {e}")
        else:
            try:
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    logger.error("Failed to open webcam (index 0)")
                    self.camera_initialized = False
                    return

                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.camera_initialized = True
                logger.info("OpenCV camera initialized successfully.")
            except Exception as e:
                self.camera_initialized = False
                logger.error(f"Error initializing OpenCV camera: {e}")

    def capture_frame(self):
        if not self.camera_initialized:
            logger.error("Camera not initialized.")
            return None

        try:
            if is_android:
                frame = self.kivy_camera.texture.pixels
                if frame is None:
                    logger.warning("Kivy Camera returned empty frame.")
                    return None
            else:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logger.warning("OpenCV Camera returned empty frame.")
                    return None

            return frame
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None

    def release(self):
        if is_android:
            if self.kivy_camera:
                self.kivy_camera.stop()
            logger.info("Kivy camera released.")
        else:
            if self.cap:
                self.cap.release()
            logger.info("OpenCV camera released.")
        self.camera_initialized = False
