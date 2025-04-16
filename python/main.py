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
        self.usb_camera = UsbCamera(0)
        self.pi_camera = UsbCamera(2)  # PiCamera()

        # Detectors
       
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)
        self.apriltag_detector = ApriltagDetector()
        self.running = True
        self.stop_seen = False
        self.last_time_seen = 0
        self.last_apriltag = (None, None) 
        self.intersection_navigator = IntersectionNavigator(self.ser)
        self.tolerance = 2
        
        width, height = self.usb_camera.width, self.usb_camera.height
        self.crosswalk_roi_top = [
            [height - (height // 5), height],
            [0, width],
        ]
        self.crosswalk_roi_bottom = [
            [height - (height // 5), height],
            [0, width],
        ]
        self.crosswalk_bottom_active =  False
        self.crosswalk_top = CrosswalkDetector()
        self.crosswalk_bottom = CrosswalkDetector()
        
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
            detected_crosswalk_top = self.crosswalk_top.detect(
                    usb_camera_frame, self.crosswalk_roi_top
                )
            
            detected_crosswalk_bottom = False
            if self.crosswalk_bottom_active:
                detected_crosswalk_bottom = self.crosswalk_top.detect(
                        usb_camera_frame, self.crosswalk_roi_bottom
                    )
                
                
               
                
         

            if detected_crosswalk_top or self.crosswalk_bottom_active:
                self.crosswalk_bottom_active = True
                ret, pi_camera_frame = self.pi_camera.cap.read()
            
                # apriltag
                label = self.apriltag_detector.detect(pi_camera_frame)
                print(label) if label != "no sign" else None
                
                if label != "no sign":
                    self.last_apriltag = (label, time.time())
                
                if detected_crosswalk_bottom:
                    self.intersection_navigator.navigate_by_tag(label)
                    continue
                    
                if self.debug:      
                    cv2.imshow("pi", pi_camera_frame)

            # lane
            if not self.crosswalk_bottom_active:
                    usb_camera_frame = self.lane_detector.detect(usb_camera_frame)

            else:
                
                self.ser.send("center 130")

                

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
