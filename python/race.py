import cv2
import time
import sys

from camera import UsbCamera, ThreadedCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
from ultrasonic import Ultrasonic
import serial_connector



class Robot:
    def __init__(self, args):
        # Check debug value
        if (
            len(args) > 2
            and args[1].lower() == "debug"
            and args[2].lower() in ["true", "1", "y", "active"]
            
        ):
            self.debug = True
        else:
            self.debug = DEBUG

        self.try_except = TRY_EXCEPT

        # Initialize serial connection
        self.ser = serial_connector.connect()

        # Initialize USB Camera
        self.usb_camera = ThreadedCamera(0)

        # Initialize lane detector
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)

        self.ultrasonic = Ultrasonic(race=True, ser=self.ser)
        

   

    def loop(self):
        """Main control loop for the robot."""
        while True:
            ret, usb_camera_frame = self.usb_camera.read()
            if not ret: 
                print("Failed to capture frame from camera")
                break
            if not self.ultrasonic.check_distance():
                # Process lane detection on the camera frame.
                usb_camera_frame = self.lane_detector.detect(usb_camera_frame)

            if self.debug:
                cv2.imshow("usb", usb_camera_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(FRAME_DELAY)

    def autorun(self):
        """Executes the robot control loop, wrapped in optional error handling."""
        if self.try_except:
            try:
                self.loop()
            except Exception as e:
                print(f"Error: {e}")
        else:
            self.loop()

        self.exit()

    def exit(self):
        """Cleans up resources before exiting."""
        self.usb_camera.cap.release()
        cv2.destroyAllWindows()
        self.ser.close()
        sys.exit()


if __name__ == "__main__":
    robot = Robot(sys.argv)
    robot.autorun()
    print("Program terminated.")
