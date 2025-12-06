from vision.camera import Camera
from vision.city_vision_processing import VisionProcessor
from vision.apriltag import ApriltagDetector
from vision.traffic_light import TrafficLightDetector
from controller import RobotController
from config_city import SPEED, CROSSWALK_SLEEP, CROSSWALK_THRESH_SPEND, default_height, default_width
import config_city 
from stream import start_stream
import logging
import cv2
import time
import threading
logging.disable(logging.DEBUG)
logger = logging.getLogger(__name__)

config_city.DEBUG = False

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()

        self.vision = VisionProcessor()
        self.apriltag_detector = ApriltagDetector()
        self.traffic_light = TrafficLightDetector()
        self.crosswalk_time_start = 0
        self.crosswalk_last_seen = 0
        self.last_tag = None
        self.stop_last_seen = None
        self.red_traffic_light_seen = 0
        
    def check_crosswalk(self, frame):
        self.check_traffic_light(frame)
        now = time.time()
        if now - self.crosswalk_last_seen>= CROSSWALK_THRESH_SPEND:
            # Only reset the crosswalk timer if it's not already running
            self.crosswalk_time_start = now
            self.crosswalk_last_seen = now

        # If crosswalk timer is running, check for elapsed time
        if self.crosswalk_time_start != 0 and self.red_traffic_light_seen == 0:
            elapsed = now - self.crosswalk_time_start
            if elapsed >= CROSSWALK_SLEEP:
                self.crosswalk_time_start = 0
                logger.debug(f"navigate with tag: {self.last_tag}")
                # Navigate based on last tag detected
                if self.last_tag == 2:
                     time.sleep(0.1)
                     self.control.forward_pulse(f"f {SPEED} 5 90 f {SPEED} 4 140")
                     time.sleep(0.1)
                elif self.last_tag == 3:
                    time.sleep(0.1)
                    self.control.forward_pulse(f"f {SPEED} 6 90 f {SPEED} 4 60")
                    time.sleep(0.1)
                elif self.last_tag == 4:
                    time.sleep(0.1)
                    self.control.forward_pulse(f"f {SPEED} 9 95")
                    time.sleep(0.1)
                else:
                    time.sleep(0.1)
                    self.control.forward_pulse(f"f {SPEED}  10 95")
                    time.sleep(0.1)
    
    def check_traffic_light(self, frame):
        now = time.time()
        if self.red_traffic_light_seen and (now - self.red_traffic_light_seen) <= 0.5:
            return
        color, debug_frame = self.traffic_light.detect(frame)
        if color == "RED":
            self.red_traffic_light_seen = now
            return
        
        self.red_traffic_light_seen = 0
        
        
    def run(self):
        logger.info("starting")
        prev_time = time.time()
        try:
            while True:
                tag = False
                if config_city.DEBUG:
                    cv2.waitKey(1)
                angle=90
                crosswalk = False
                
                if self.crosswalk_time_start == 0: # 3 sec
                    frame_at = self.camera.capture_frame(resize=False)

                    frame = cv2.resize(frame_at, (default_width, default_height), interpolation=cv2.INTER_AREA)

                    result = self.vision.detect(frame)
        
                    angle = result.get("steering_angle")
            
                    crosswalk = result.get("crosswalk", False)
                    
                    tags, frame_at, largest_tag = self.apriltag_detector.detect(frame_at)
                    
                    if largest_tag is not None:
                        tag_id = largest_tag["id"]
                        if largest_tag["corners"][1][1] > 180:  
                            if tag_id == 5:
                                    tag = True
                                    self.stop_last_seen = time.time()

                            self.last_tag = tag_id       
                                      
                    if tag or (self.stop_last_seen is not None and time.time() - self.stop_last_seen <= 1):
                        self.control.stop()
                        time.sleep(0.01)
                        continue
                
                    if config_city.DEBUG: 
                        debug = result.get("debug") or {}
                        if debug.get("combined") is not None:
                            cv2.imshow("combined", debug.get("combined"))
                        if frame is not None:
                            cv2.imshow("frame", frame)
                        if frame_at is not None:
                            cv2.imshow("at", frame_at)
             
                    if config_city.STREAM:
                        curr_time = time.time()
                        fps = 1.0 / (curr_time - prev_time)
                        prev_time = curr_time
                        display_frame = debug.get("combined").copy()
                        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        config_city.debug_frame_buffer = display_frame
                    
                else: # not 3 sec
                    self.control.stop()
                    time.sleep(0.1)
                    frame_at = self.camera.capture_frame(resize=False)
                    self.check_crosswalk(frame_at)
                    if config_city.STREAM:
                        config_city.debug_frame_buffer = frame_at
                    
                    continue
                

                if crosswalk and time.time() - self.crosswalk_last_seen >= CROSSWALK_THRESH_SPEND:
                    self.control.stop()
                    time.sleep(0.1)
                    self.check_crosswalk(frame_at)
                    continue
                
                self.control.set_angle(angle)
                time.sleep(0.01)
                self.control.set_speed(SPEED)  
                time.sleep(0.01)
                
                

        except KeyboardInterrupt:
            logger.error("error KeyboardInterrupt")
            
        except Exception as e:
            logger.error(f"error {e}")
        finally:
            
            self.close()
            logger.info("exited")
    
    def close(self):
        self.control.stop()
        self.control.set_angle(90)
        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    if config_city.STREAM:
        flask_thread = threading.Thread(target=start_stream, daemon=True)
        flask_thread.start()
    robot = Robot()
    robot.run()