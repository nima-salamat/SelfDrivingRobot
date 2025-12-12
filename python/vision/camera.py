from base_config import default_width, default_height, CAMERA_MODE
try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

from time import sleep
import cv2
import threading
import logging

logger = logging.getLogger(__name__)

class Camera:
    def __init__(self, width=default_width, height=default_height, mode=CAMERA_MODE):
        self.width = width
        self.height = height
        self.mode = mode
        self.pi_mode = False
        self.camera_initialized = False

        # Initialize camera
        if mode == "picam" and Picamera2 is not None:
            self.pi_mode = True
            try:
                self.picam = Picamera2()
                self.setup_camera()
                logger.info("Using Picamera2")
            except Exception as e:
                logger.error(f"Picamera2 init failed: {e}")
                logger.info("Falling back to OpenCV")
                self.pi_mode = False
                self.cap = cv2.VideoCapture(0)
                self.setup_camera()
        else:
            self.pi_mode = False
            self.cap = cv2.VideoCapture(0)
            self.setup_camera()
            logger.info("Using OpenCV VideoCapture")

    def setup_camera(self):
        if self.pi_mode:
            try:
                config = self.picam.create_preview_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"}
                )
                self.picam.configure(config)
                self.picam.start()
                sleep(2)
                self.camera_initialized = True
            except Exception as e:
                logger.error(f"Picamera2 setup failed: {e}")
                self.camera_initialized = False
                raise
        else:
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                logger.error("Failed to open webcam")
                raise RuntimeError("No webcam detected.")

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Test capture
            for _ in range(5):
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.camera_initialized = True
                    break
                sleep(0.1)

            if not self.camera_initialized:
                logger.error("Webcam test capture failed")
                raise RuntimeError("Webcam not functioning properly")

    def capture_frame(self, resize=True):
        if not self.camera_initialized:
            logger.error("Camera not initialized")
            return None

        try:
            if self.pi_mode:
                frame = self.picam.capture_array()
                if frame is None or frame.size == 0:
                    logger.warning("Picamera2 returned empty frame")
                    return None
            else:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logger.warning("OpenCV camera returned no frame")
                    return None

            if resize:
                if frame.shape[:2] != (self.height, self.width):
                    frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

            return frame
        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None

    def release(self):
        self.camera_initialized = False
        if self.pi_mode:
            try:
                self.picam.stop()
                self.picam.close()
            except Exception as e:
                logger.error(f"Error releasing Picamera2: {e}")
        else:
            try:
                self.cap.release()
            except Exception as e:
                logger.error(f"Error releasing OpenCV camera: {e}")


class CameraThreaded(Camera):
    def __init__(self, manager_dict):
        super().__init__(width=default_width, height=default_height, mode=CAMERA_MODE)
        self.thread = threading.Thread(target=self.reader, args=(manager_dict,), daemon=True)
        self.thread.start()

    def reader(self, manager_dict):
        while True:
            frame = self.capture_frame(resize=False)
            manager_dict["frame"] = frame
