import cv2
import time
import threading
import sys

from camera import UsbCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
import serial_connector
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory


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
        self.usb_camera = UsbCamera(0)

        # Initialize lane detector
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=False)

        # Set up the ultrasonic sensor via gpiozero with pigpio backend.
        factory = PiGPIOFactory()
        self.ultrasonic = DistanceSensor(echo=17, trigger=4, pin_factory=factory)
        self.min_dist = 0.3

        # Latest ultrasonic distance value
        self.ultrasonic_distance = 1.0

        # Start the ultrasonic update thread
        self.ultrasonic_thread = threading.Thread(
            target=self._update_ultrasonic_distance, daemon=True
        )
        self.ultrasonic_thread.start()

    def _update_ultrasonic_distance(self):
        """Continuously updates the ultrasonic distance measurement."""
        while True:
            self.ultrasonic_distance = self.ultrasonic.distance
            time.sleep(0.1)  # Update every 100ms

    def handle_pass_block(self):
        """Executes a maneuver to pass a block."""
        self.ser.send("sharp left 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(0.8)

        self.ser.send("sharp left 160")
        time.sleep(0.8)

    def loop(self):
        """Main control loop for the robot."""
        while True:
            ret, usb_camera_frame = self.usb_camera.cap.read()
            if not ret:
                print("Failed to capture frame from camera")
                break

            # Check ultrasonic distance and react accordingly.
            if self.min_dist <= self.ultrasonic_distance <= self.min_dist + 0.05:
                self.ser.send("center 0")
                time.sleep(0.5)
                self.handle_pass_block()
            elif self.ultrasonic_distance < self.min_dist:
                self.ser.send("center 0")
                time.sleep(0.3)
                while self.ultrasonic_distance <= self.min_dist + 0.1:
                    self.ser.send("center -0")
                    time.sleep(0.1)

            # Process lane detection on the camera frame.
            processed_frame = self.lane_detector.detect(usb_camera_frame)

            if self.debug:
                cv2.imshow("usb", processed_frame)
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
