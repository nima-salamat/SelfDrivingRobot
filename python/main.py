from vision.camera import Camera
from vision.vision_processing import VisionProcessor
from vision.apriltag import ApriltagDetector
from controller import RobotController
from utils.crosswalk_navigation import Navigate
from config import DEBUG, SPEED, CROSSWALK_SLEEP, CROSSWALK_THRESH_SPEND
import logging
import cv2
import time
logging.disable(logging.DEBUG)
logger = logging.getLogger(__name__)


DEBUG = True

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()
        self.vision = VisionProcessor()
        self.apriltag_detector = ApriltagDetector()
        self.nav = Navigate(self.control)
        self.crosswalk_time_start = 0
        self.crosswalk_last_seen = 0
        self.last_tag = None
        
    def check_crosswalk(self, frame):
        if self.crosswalk_last_seen - time.time() > CROSSWALK_THRESH_SPEND:
            # Only reset the crosswalk timer if it's not already running
            if self.crosswalk_time_start == 0:
                self.crosswalk_time_start = time.time()
            self.crosswalk_last_seen = time.time()

        # If crosswalk timer is running, check for elapsed time
        if self.crosswalk_time_start != 0:
            elapsed = time.time() - self.crosswalk_time_start
            if elapsed >= CROSSWALK_SLEEP:
                self.crosswalk_time_start = 0

                # Navigate based on last tag detected
                if self.last_tag == 0:
                    self.nav.right()
                elif self.last_tag == 1:
                    self.nav.left()
                elif self.last_tag == 2:
                    self.nav.straight()
                else:
                    self.nav.straight()

            else:
                # Attempt to read AprilTag while the timer is running
                tags = self.apriltag_detector.detect(frame)
                if tags:
                    first_tag = tags[0]
                    tag_id = first_tag["id"]
                    self.last_tag = tag_id
                return True

        return False

                    
    def run(self):
        logger.info(f"starting . . . ")
        try:
            while True:
                angle=90
                if self.crosswalk_time_start == 0:
                    frame = self.camera.capture_frame()
                    
                    result = self.vision.detect(frame)
        
                    angle = result.get("steering_angle")
            
                    crosswalk = result.get("crosswalk", False)
                    if DEBUG:
                        cv2.imshow("", result.get("debug").get("combined"))
                        cv2.imshow("frame", frame)

                        if cv2.waitKey(1) == ord('q'):
                            break
                else:
                    frame = self.camera.capture_frame(resize=False)
                # print(type(result.get('debug')))
                
                    
                if self.check_crosswalk(frame):
                    continue 
                
                self.control.set_angle(angle)
                # idk should i send speed too?
                self.control.set_speed(SPEED)     

        except KeyboardInterrupt:
            logger.error("error KeyboardInterrupt")
        except Exception as e:
            logger.error(f"error {e}")
            print(e)
        finally:
            
            self.close()
            logger.info(f"exited . . . ")
            
    
    def close(self):
        self.camera.release()
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    robot = Robot()
    robot.run()