import cv2
import time
from camera import UsbCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
from crosswalk_detection import CrosswalkDetector
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
        self.usb_camera = UsbCamera(0)
        self.pi_camera = UsbCamera(2)  # PiCamera()

        # Detectors
       
        self.crosswalk_detector = CrosswalkDetector(debug=self.debug, ser=self.ser)
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)
        self.apriltag_detector = ApriltagDetector()
        self.running = True
        self.stop_seen = False
        self.last_time_seen = 0
        self.last_apriltag = (None, None) 
        self.intersection_navigator = IntersectionNavigator(self.ser)
        self.tolerance = 2
        
        if "no-stop" in args:
            self.running = False
        

    def loop(self):
        while True:
            if self.running:
                # camera read
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
            
            
            # camera read
            ret, usb_camera_frame = self.usb_camera.cap.read()
            # crosswalk
            usb_camera_frame, detected_crosswalk, time_ = (
                self.crosswalk_detector.detect(
                    usb_camera_frame, self.usb_camera.crosswalk_roi
                )
            )
                

            if detected_crosswalk:
                ret, pi_camera_frame = self.pi_camera.cap.read()
            
                # apriltag
                label = self.apriltag_detector.detect(pi_camera_frame)
                print(label) if label != "no sign" else None
                
                if label != "no sign":
                    self.last_apriltag = ()
                    
                
                if self.debug:      
                    cv2.imshow("pi", pi_camera_frame)

            # lane
            if not detected_crosswalk:
                usb_camera_frame = self.lane_detector.detect(usb_camera_frame)

            else:
                if time_ + 3.1 < time.time():
                    self.ser.send("center 120")
                else:
                    self.ser.send("center 0")

                if (
                    self.last_apriltag[0] is not None
                    and (time.time() - self.last_apriltag[1]) < self.tolerance
                ):
                    # Navigate
                    self.intersection_navigator.navigate_by_tag(label)
                    self.last_apriltag = (None, None)

            if self.debug:
                cv2.imshow("usb", usb_camera_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(FRAME_DELAY)

    def autorun(self):
        if self.try_except:
            try:
                self.loop()
            except Exception as e:
                print(e)

        else:
            self.loop()

        self.exit()

    def exit(self):
        self.usb_camera.cap.release()
        self.pi_camera.cap.release()
        cv2.destroyAllWindows()
        self.ser.close()
        sys.exit()


if __name__ == "__main__":
    robot = Robot(sys.argv)
    robot.autorun()
    print("Program terminated.")
