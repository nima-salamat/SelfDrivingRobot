from vision.camera import Camera
from vision.race import VisionProcessor
from vision.apriltag import ApriltagDetector
from controller import RobotController
from config import SPEED, CROSSWALK_SLEEP, CROSSWALK_THRESH_SPEND
import config
import logging
import cv2
import time
logging.disable(logging.DEBUG)
logger = logging.getLogger(__name__)

config.DEBUG = False

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()

        self.vision = VisionProcessor()
        self.apriltag_detector = ApriltagDetector()

    def run(self):
        logger.info("starting")
        try:
            while True:
                if config.DEBUG:
                    cv2.waitKey(1)
            
                frame = self.camera.capture_frame()
                
                result = self.vision.detect(frame)

                angle = result.get("steering_angle")
        
                if config.DEBUG:
                    debug = result.get("debug") or {}
                    if debug.get("combined") is not None:
                        cv2.imshow("combined", debug.get("combined"))
                    if frame is not None:
                            cv2.imshow("frame", frame)

                
                self.control.set_angle(angle)
                time.sleep(0.01)
                self.control.set_speed(SPEED)  
                time.sleep(0.01)

        except KeyboardInterrupt:
            logger.error("error KeyboardInterrupt")
        except Exception as e:
            logger.error(f"error {e}")
            print(e)
        finally:
            
            self.close()
            logger.info("exited")
            
    
    def close(self):
        time.sleep(0.01)
        self.control.stop()
        time.sleep(0.01)
        self.control.set_angle(90)
        time.sleep(0.01)
        self.camera.release()
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    robot = Robot()
    robot.run()