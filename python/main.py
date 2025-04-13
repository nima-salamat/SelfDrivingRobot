import cv2
import time
from camera import UsbCamera, PiCamera, CameraDevices
from config import FRAME_DELAY
from lane_detection import LaneDetector
from crosswalk_detection import CrosswalkDetector
from trafficlight_detection import TrafficLightDetector
from apriltag_detection import ApriltagDetector
import usb.core
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
            self.debug = False

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
        self.trafficlight_detector = TrafficLightDetector(
            self.pi_camera.width,
            self.pi_camera.height,
            normalized_roi=[[0.5, 0], [0.5, 0]],
        )
        self.crosswalk_detector = CrosswalkDetector(debug=self.debug)
        self.lane_detector = LaneDetector(self.usb_camera, self.ser, debug=self.debug)
        self.apriltag_detector = ApriltagDetector()

    def autorun(self):
        try:
            while True:
                ret, usb_camera_frame = self.usb_camera.cap.read()
                
                usb_camera_frame, detected_crosswalk, time_ = (
                    self.crosswalk_detector.detect(
                        usb_camera_frame, self.usb_camera.crosswalk_roi, self.ser
                    )
                )

                if detected_crosswalk:
                    ret, pi_camera_frame = self.pi_camera.cap.read()
                    # trafficlight
                    pi_camera_frame, color = self.trafficlight_detector.detect(
                        pi_camera_frame
                    )
                    print(color) if color != "no light" else None  # center 0
                    # apriltag
                    label = self.apriltag_detector.detect(pi_camera_frame)
                    print(label) if label != "no sign" else None

                    if label != "no sign":
                        self.ser.send(
                            "intersection" + " " + label + " " + color
                        )  # --- attention!! not implemented yet. . .

                # lane
                if not detected_crosswalk:
                    usb_camera_frame = self.lane_detector.detect(usb_camera_frame)

                else:
                    
                    if time_ + 4 < time.time():
                        self.ser.send("center 100")
                    else:
                        self.ser.send("center 0")

                if self.debug:
                    cv2.imshow("usb", usb_camera_frame)
                # cv2.imshow("pi",pi_camera_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                time.sleep(FRAME_DELAY)
        except Exception:
            pass
        finally:
            self.exit()

    def exit(self):
        self.usb_camera.cap.release()
        self.pi_camera.cap.release()
        self.ser.close()


if __name__ == "__main__":
    robot = Robot(sys.argv)
    robot.autorun()
    print("Program terminated.")
