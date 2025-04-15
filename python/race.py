import cv2
import time
from camera import UsbCamera
from config import FRAME_DELAY, DEBUG, TRY_EXCEPT
from lane_detection import LaneDetector
import serial_connector
import sys
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory


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

        # enable or disable ultersonic
        # self.ser.send("ultrasonic on" if ULTERASONIC_ENABLED else "ultrasonic off")

        self.usb_camera = UsbCamera(0)

        # Detectors
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=False)

        factory = PiGPIOFactory()
        self.ultrasonic = DistanceSensor(echo=17, trigger=4, pin_factory=factory)
        self.min_dist = 0.3

    def handle_pass_block(self):
        self.ser.send("sharp left 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(0.8)

        self.ser.send("sharp left 160")
        time.sleep(0.8)

    def loop(self):
        while True:
            ret, usb_camera_frame = self.usb_camera.cap.read()
            if self.min_dist <= self.ultrasonic.distance <= self.min_dist + 0.05:
                self.ser.send("center 0")
                time.sleep(0.5)
                self.handle_pass_block()
            elif self.ultrasonic.distance < self.min_dist:
                self.ser.send("center 0")
                time.sleep(0.3)
                while self.ultrasonic.distance <= self.min_dist + 0.1:
                    self.ser.send("center -150")

            usb_camera_frame = self.lane_detector.detect(usb_camera_frame)
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
        cv2.destroyAllWindows()
        self.ser.close()
        sys.exit()


if __name__ == "__main__":
    robot = Robot(sys.argv)
    robot.autorun()
    print("Program terminated.")
