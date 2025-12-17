import logging
import time
import cv2
import numpy as np
from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from vision.camera import Camera
from vision.vision_processing import VisionProcessor
from controller import RobotController
from base_config import SPEED

logging.disable(logging.DEBUG)
logger = logging.getLogger(__name__)

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()
        self.vision = VisionProcessor()

    def run(self):
        logger.info("starting")
        try:
            while True:
                frame = self.camera.capture_frame()
                result = self.vision.detect(frame)
                angle = result.get("steering_angle")
                debug_frame = result["debug"]["combined"]
                self.control.set_angle(angle)
                self.control.set_speed(SPEED)
                return debug_frame

        except KeyboardInterrupt:
            logger.error("error KeyboardInterrupt")
        except Exception as e:
            logger.error(f"error {e}")
        finally:
            self.close()

    def close(self):
        self.camera.release()

class RobotApp(App):
    def build(self):
        self.robot = Robot()

        self.layout = BoxLayout(orientation='vertical')

        self.img = Image()
        self.layout.add_widget(self.img)

        Clock.schedule_interval(self.update, 1.0 / 30.0)  # 30 FPS

        return self.layout

    def update(self, dt):
        debug_frame = self.robot.run()

        if debug_frame is not None:
            buf = debug_frame.tobytes()
            texture = self.img.texture
            texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            self.img.texture = texture

if __name__ == '__main__':
    RobotApp().run()
