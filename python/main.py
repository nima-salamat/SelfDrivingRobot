import cv2
import time
from camera import UsbCamera, PiCamera
from config import FRAME_DELAY
from lane_detection import LaneDetector
from crosswalk_detection import CrosswalkDetector
from trafficlight_detection import TrafficLightDetector
from apriltag_detection import ApriltagDetector
import serial_connector


class Robot:
    def __init__(self):
        # Serial
        self.ser = serial_connector.connect()
        
        # Cameras
        self.usb_camera = UsbCamera(0)  # bottom

        self.pi_camera = UsbCamera(1) # PiCamera()
        # Detectors
        self.trafficlight_detector = TrafficLightDetector(
            self.pi_camera.width,
            self.usb_camera.height,
            normalized_roi=[[0, 0.5], [0, 0.5]],
        )
        self.crosswalk_detector = CrosswalkDetector()
        self.lane_detector = LaneDetector(self.pi_camera, self.ser)
        self.apriltag_detector = ApriltagDetector()

    def autorun(self):
        
        while True:
            ret, pi_camera_frame = self.pi_camera.cap.read()
            ret, usb_camera_frame = self.usb_camera.cap.read()

           
            usb_camera_frame, detected_crosswalk = self.crosswalk_detector.detect(usb_camera_frame, self.pi_camera.crosswalk_roi, self.ser)
            
            if detected_crosswalk:

                # trafficlight
                pi_camera_frame, color = self.trafficlight_detector.detect(pi_camera_frame)
                # print(color) if color !="no light" else None # center 0
                # apriltag
                label = self.apriltag_detector.detect(pi_camera_frame)
                print(label) if label !="no sign" else None
                
                if label != "no sign":
                    self.ser.send("intersection"+" "+ label)  # --- attention!! not implemented yet. . .                  

            # lane
            if not detected_crosswalk:
                camera_bottom_frame = self.lane_detector.detect(usb_camera_frame)
            else:
                self.ser.send("center")
                
            cv2.imshow("",camera_bottom_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(FRAME_DELAY)

        self.exit()
    
    def exit(self):
        self.usb_camera.cap.release()
        self.pi_camera.cap.release()
        self.ser.close()


if __name__ == "__main__":
    robot = Robot()
    robot.autorun()
    print("Program terminated.")