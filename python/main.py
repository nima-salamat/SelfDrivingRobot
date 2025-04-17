import cv2
import time
from camera import UsbCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
from crosswalk_detection_v2 import CrosswalkDetector
from apriltag_detection import ApriltagDetector
from cross_intersection import IntersectionNavigator
import serial_connector
import sys


class Robot:
    def __init__(self, args):
        # Check debug value
        if (
            len(args) > 2
            and args[1] in ["debug", "DEBUG", "Debug"]
            and args[2] in ["True", "true", "1", "y", "active"]
        ):
            self.debug = True
        else:
            self.debug = DEBUG

        self.try_except = TRY_EXCEPT

        # Serial
        self.ser = serial_connector.connect()

        # Cameras
        # try:
        #     self.usb_camera = UsbCamera(CameraDevices.get_address(0))
        #     # "D:/div_5/open_cv/video_2025-02-15_11-43-46.mp4"
        #     self.pi_camera = UsbCamera(CameraDevices.get_address(1)) # PiCamera()
        # except usb.core.NoBackendError:
        # Cameras
        self.usb_camera = UsbCamera(1)
        self.pi_camera = UsbCamera(0)  # PiCamera()

        # Detectors

        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)
        self.apriltag_detector = ApriltagDetector()
        self.running = True
        self.stop_seen = False
        self.last_time_seen = 0
        self.last_apriltag = (None, None)
        self.intersection_navigator = IntersectionNavigator(self.ser)
        self.tolerance = 2

        self.crosswalk_detector = CrosswalkDetector()
        width, height = self.usb_camera.width, self.usb_camera.height
        self.crosswalk_roi = [[0, height],[width//5, width - width//5]]
        if "no-stop" in args:
            self.running = False

    def loop(self):
        while True:
            if self.running:
                ret, pi_camera_frame = self.pi_camera.cap.read()
                label = self.apriltag_detector.detect(pi_camera_frame)
                if label == "stop":
                    self.stop_seen = True
                    self.last_time_seen = time.time()
                elif (
                    label == "no sign"
                    and self.stop_seen
                    and time.time() - self.last_time_seen > 1
                ):
                    self.running = False
                self.ser.send("center 0")
                continue

            # Read USB camera frame
            ret, usb_camera_frame = self.usb_camera.cap.read()
            if not ret:
                print("Failed to read USB camera frame")
                continue

            # Detect crosswalk in bottom half (misnamed as crosswalk_roi_top)
            detected_crosswalk = self.crosswalk_detector.detect(
                usb_camera_frame, self.crosswalk_roi
            )

          

            if detected_crosswalk:
                
                self.ser.send("center 0")
                self.ser.send("slow right 130")
                time.sleep(0.5)
                self.ser.send("center 0")

                print("crosswalk stoped")
                for i in range(6):
                    
                    label = "no sign"
                    ret, pi_camera_frame = self.pi_camera.cap.read()
                    label = self.apriltag_detector.detect(pi_camera_frame)
                    if label != "no sign":
                        print(f"AprilTag detected: {label}")
                        self.last_apriltag = (label, time.time())
                    time.sleep(0.5)
                
                if self.last_apriltag[0] is None:
                    print("could not find apriltag")
                elif self.last_apriltag is not None and self.last_apriltag[1] + 1 > time.time():
                                        
                    self.intersection_navigator.navigate_by_tag(self.last_apriltag[0])
                    
                
                

            # Lane detection or default command
            if not detected_crosswalk:
                usb_camera_frame = self.lane_detector.detect(usb_camera_frame)
            else:
                self.ser.send("center 0")

            # Show the frame with detections if debug is on
            if self.debug:
                cv2.imshow("usb", usb_camera_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(FRAME_DELAY)


if __name__ == "__main__":
    Robot(sys.argv).loop()
