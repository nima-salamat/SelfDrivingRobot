import cv2
try:
    from picamera2 import Picamera2
except Exception:
    print("we cant import picamera2 in windows :D")
import time
import usb.core
import usb.util
import threading
from config import (TARGET_FPS,
                    ROI_Y_START_FRAC,
                    ROI_X_START_FRAC,
                    ROI_X_END_FRAC,
                    LANE_WIDTH_FRAC,
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT, 
                    FRAME_DELAY
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
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  

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
            0,
            (3 * self.height) // 5,
            self.width // 2,
            (2 * self.height) // 5,
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
        

class CameraDevices:
    camera_name_index = {0: "webcam c270", 1: "sony playstation eye"}
    camera_devices = {"webcam c270":None, "sony playstation eye":None}
    
    @classmethod
    def find_device_by_name(cls, device_name):
        # Find all connected USB devices
        devices = usb.core.find(find_all=True)
        
        for dev in devices:
            try:
                # Get the device's product name
                product = usb.util.get_string(dev, dev.iProduct)
                
                # Check if the name matches (case insensitive)
                if product and device_name.lower() in product.lower():
                    print(f"Found device: {product} (VendorID: 0x{dev.idVendor:04x}, ProductID: 0x{dev.idProduct:04x})")
                    cls.camera_devices[device_name] = dev.address
                    return dev
                    
            except (usb.core.USBError, ValueError, IndexError) as e:
                continue
        
        return None
    
    @classmethod
    def are_activated(cls):
        if all([cls.find_device_by_name(name) for name in cls.camera_devices]):
            return True
        return False
    
    @classmethod
    def get_address(cls, index):
        device_name = cls.camera_name_index[index]
        cls.find_device_by_name(device_name)
        return cls.camera_devices.get(device_name)




class ThreadedCamera(UsbCamera):
    def __init__(self, src=0):
        super().__init__(src)
        
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()
            time.sleep(FRAME_DELAY)

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True


