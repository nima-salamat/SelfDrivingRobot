import logging
import numpy as np
from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from vision.camera import Camera
from vision.vision_processing import VisionProcessor
from controller import RobotController
from kivy.graphics.texture import Texture
from base_config import SPEED
import time

logging.disable(logging.DEBUG)
logger = logging.getLogger(__name__)

class Robot:
    def __init__(self):
        self.camera = Camera()
        self.control = RobotController()
        self.vision = VisionProcessor()

    def run(self):
        try:
            frame = self.camera.capture_frame()
            if frame is None:
                return None
            result = self.vision.detect(frame)
            angle = result.get("steering_angle")
            debug_frame = result["debug"]["combined"]
            time.sleep(0.01)
            self.control.set_angle(angle)
            time.sleep(0.01)
            self.control.set_speed(SPEED)
            time.sleep(0.01)
            return debug_frame
        except Exception as e:
            logger.error(f"Robot run error: {e}")
            return None

    def close(self):
        self.camera.release()
        self.control.connection.close()


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
        if debug_frame is None:
            return

        frame_rgb = debug_frame[:, :, ::-1]  # BGR -> RGB
        buf = frame_rgb.tobytes()

        if not self.img.texture:
            self.img.texture = Texture.create(
                size=(frame_rgb.shape[1], frame_rgb.shape[0]),
                colorfmt='rgb'
            )
            self.img.texture.flip_vertical()

        self.img.texture.blit_buffer(buf, colorfmt='rgb', bufferfmt='ubyte')
        self.img.canvas.ask_update()


if __name__ == '__main__':
    RobotApp().run()
