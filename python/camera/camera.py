import cv2
import time
import usb.core
import usb.util
import threading
try:
    from picamera2 import Picamera2, Transform
except ImportError:
    print("We can't import picamera2 on Windows :D")

from config import (
    TARGET_FPS, ROI_Y_START_FRAC, ROI_X_START_FRAC, ROI_X_END_FRAC,
    LANE_WIDTH_FRAC, CAMERA_WIDTH, CAMERA_HEIGHT, FRAME_DELAY
)


def initialize_camera(index=0, width=480, height=240):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError("Error: Could not open camera.")

    # Set camera parameters
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Verify resolution
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Error reading from camera.")

    h, w = frame.shape[:2]
    print(f"Camera resolution: {w}x{h}")
    return cap, w, h


class Camera:
    def __init__(self, index=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        self.cap, self.width, self.height = initialize_camera(index, width, height)


class UsbCamera(Camera):
    def __init__(self, index=0):
        super().__init__(index)
        self._init_roi()

    def _init_roi(self):
        self.crosswalk_roi = (0, (3 * self.height) // 5, self.width // 2, (2 * self.height) // 5)
        self.ROI_Y_START = int(self.height * ROI_Y_START_FRAC)
        self.ROI_Y_END = self.height
        self.ROI_X_START = int(self.width * ROI_X_START_FRAC)
        self.ROI_X_END = int(self.width * ROI_X_END_FRAC)
        self.ROI_WIDTH = self.ROI_X_END - self.ROI_X_START
        self.LANE_WIDTH = int(self.width * LANE_WIDTH_FRAC)

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, None
        roi = frame[self.ROI_Y_START:self.ROI_Y_END, self.ROI_X_START:self.ROI_X_END]
        return True, roi


class PiCamera:
    def __init__(self):
        self.picam = Picamera2()
        config = self.picam.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
            transform=Transform(hflip=True, vflip=True)
        )
        self.picam.configure(config)
        self.picam.start()
        self.cap = self.picam
        self.width, self.height = CAMERA_WIDTH, CAMERA_HEIGHT


class CameraDevices:
    _device_cache = None
    camera_name_index = {0: "webcam c270", 1: "sony playstation eye"}
    camera_devices = {name: None for name in camera_name_index.values()}

    @classmethod
    def _get_devices(cls):
        if cls._device_cache is None:
            cls._device_cache = list(usb.core.find(find_all=True))
        return cls._device_cache

    @classmethod
    def find_device_by_name(cls, device_name):
        for dev in cls._get_devices():
            try:
                product = usb.util.get_string(dev, dev.iProduct)
                if product and device_name.lower() in product.lower():
                    cls.camera_devices[device_name] = dev.address
                    return dev
            except (usb.core.USBError, ValueError, IndexError):
                continue
        return None

    @classmethod
    def are_activated(cls):
        return all(cls.find_device_by_name(name) for name in cls.camera_devices)

    @classmethod
    def get_address(cls, index):
        name = cls.camera_name_index[index]
        cls.find_device_by_name(name)
        return cls.camera_devices.get(name)


class ThreadedCamera(UsbCamera):
    def __init__(self, src=0):
        super().__init__(src)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()
            # Optional: tune sleep if CPU usage is too high
            if FRAME_DELAY > 0:
                time.sleep(FRAME_DELAY)

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
