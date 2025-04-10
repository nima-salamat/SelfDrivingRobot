import cv2
import time
from camera import CameraBottom, Camera
from config import FRAME_DELAY
from lane_detection import LaneDetector
from crosswalk_detection import process_frame_for_crosswalk
from trafficlight_detection import TrafficLightDetector
from apriltag_detection import apriltag_detection
import serial_connector


class Robot:
    def __init__(self):
        # Serial
        self.ser = serial_connector.connect()
        
        # Cameras
        self.top_camera = Camera(1) # top --- attention!!! --- not implemented!!!
        self.bottom_camera = CameraBottom(0) # bottom
        
        # Detectors
        self.trafficlight_detector = TrafficLightDetector(
            self.bottom_camera.width,
            self.top_camera.height,
            normalized_roi=[[0, 0.5], [0, 0.5]],
        )
        self.lane_detector = LaneDetector(self.bottom_camera, self.ser)

    def autorun(self):
        
        while True:
            ret, camera_bottom_frame = self.bottom_camera.cap.read()
            # crosswalk
            camera_bottom_frame, crosswalk_detected = process_frame_for_crosswalk(
                camera_bottom_frame, self.bottom_camera.crosswalk_roi, self.ser
            )
            
            
            # trafficlight
            camera_bottom_frame, color = self.trafficlight_detector.trafficlight_detection(camera_bottom_frame)
            print(color) if color !="no light" else None # center 0
            # apriltag
            label = apriltag_detection(camera_bottom_frame)
            print(label) if label !="no sign" else None

            # lane
            camera_bottom_frame = self.lane_detector.detect(camera_bottom_frame)
            cv2.imshow("",camera_bottom_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(FRAME_DELAY)

        self.exit()
    
    def exit(self):
        self.top_camera.cap.release()
        self.bottom_camera.cap.release()
        self.ser.close()


if __name__ == "__main__":
    robot = Robot()
    robot.autorun()
    print("Program terminated.")
