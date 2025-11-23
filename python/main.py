from vision.camera import Camera
from vision.vision_processing_segment_detector_prespective_crosswalk import VisionProcessor
from controller import RobotController
from config import DEBUG, SPEED, CROSSWALK_SLEEP
import logging
import cv2
import time
logging.disable(logging.CRITICAL)
logger = logging.getLogger(__name__)


DEBUG = True

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()
        self.vision = VisionProcessor()
        self.crosswalk_time_start = 0
        
        
    def crosswalk_seen(self):
        if self.crosswalk_time_start != 0:
            self.crosswalk_time_start = time.time()
        
        elapsed = time.time() - self.crosswalk_time_start
        if elapsed >= CROSSWALK_SLEEP:
            self.crosswalk_time_start = 0
            
    def run(self):
        logger.info(f"starting . . . ")
        
        
        try:
            while True:
                
                frame = self.camera.capture_frame()
                result = self.vision.detect(frame)
                # result =  {
                #     "steering_angle": steering_angle,
                #     "error": error,
                #     "lane_type": lane_type,
                #     "debug": debug
                # }
                angle = result.get("steering_angle")
            
                crosswalk = result.get("crosswalk", False)
                if DEBUG:
                    cv2.imshow("", result.get("debug").get("combined"))
                    cv2.imshow("frame", frame)

                    if cv2.waitKey(1) == ord('q'):
                        break
                # print(type(result.get('debug')))
                if not crosswalk:
                    self.control.set_angle(angle)
                    # idk should i send speed too?
                    self.control.set_speed(SPEED)
                else:
                    self.control.set_speed(0)
                    self.control.stop()
                    
                

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