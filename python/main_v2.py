import cv2
import time
from camera import UsbCamera, ThreadedCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
from crosswalk_detection_v2 import CrosswalkDetector
from apriltag_detection import ApriltagDetector
from cross_intersection import IntersectionNavigator
from ultrasonic import Ultrasonic
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
        self.usb_camera = ThreadedCamera(0)
        self.second_camera = ThreadedCamera(2)  # PiCamera()

        # Detectors
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)
        self.apriltag_detector = ApriltagDetector()
        self.running = True
        self.stop_seen = False
        self.last_time_seen = 0
        self.last_apriltag = (None, None)
        self.intersection_navigator = IntersectionNavigator(self.ser)
        self.tolerance = 2

        self.ultrasonic = Ultrasonic(race=True, ser=self.ser)
        self.crosswalk_detector = CrosswalkDetector()
        width, height = self.usb_camera.width, self.usb_camera.height

        self.crosswalk_roi = [[0, height], [width // 5, width - width // 5]]
        if "no-stop" in args:
            self.running = False

        self.last_crosswalk_time = 0
        self.crosswalk_cooldown = 5

    def cross_walk_handler(self):
        current_time = time.time()

        if current_time > self.last_crosswalk_time + self.crosswalk_cooldown:
            self.last_crosswalk_time = current_time
            self.ser.send("sharp right 130")
            time.sleep(0.5)
            self.ser.send("center 0")
            time.sleep(1)
            print("crosswalk stopped")
            start_time = time.time()
            while time.time() - start_time < 2.7:
                ret, second_camera_frame = self.second_camera.read()
                if ret:
                    label = self.apriltag_detector.detect(second_camera_frame)
                    if label != "no sign":
                        print(f"AprilTag detected: {label}")
                        self.last_apriltag = (label, time.time())
                        break
                time.sleep(0.1)

            if (
                self.last_apriltag[0] is not None
                and self.last_apriltag[1] + 1 > time.time()
            ):
                self.intersection_navigator.navigate_by_tag(self.last_apriltag[0])
            else:
                print("could not find apriltag or detection is too old")

    def lane_detection_handler(self, usb_camera_frame, detected_crosswalk):
        if not detected_crosswalk:
            usb_camera_frame = self.lane_detector.detect(usb_camera_frame)

    def loop(self):
        while True:
            # Read
            ret, second_camera_frame = self.second_camera.read()
            label = self.apriltag_detector.detect(second_camera_frame)
            if label == "stop":
                self.ser.send("center 0")
                time.sleep(10)
                self.last_time_seen = time.time()

            self.ultrasonic.check_distance()

            # Read USB camera framea
            ret, usb_camera_frame = self.usb_camera.read()
            if not ret:
                print("Failed to read USB camera frame")
                continue

            # Detect crosswalk in bottom half
            detected_crosswalk = self.crosswalk_detector.detect(
                usb_camera_frame, self.crosswalk_roi
            )

            if detected_crosswalk:
                self.cross_walk_handler()

            # Lane detection or default command
            self.lane_detection_handler(usb_camera_frame, detected_crosswalk)

            # Show the frame with detections if debug is on
            if self.debug:
                cv2.imshow("usb", usb_camera_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(FRAME_DELAY)


if __name__ == "__main__":
    Robot(sys.argv).loop()
