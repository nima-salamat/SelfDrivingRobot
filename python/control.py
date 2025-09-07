from config import current_angle, current_speed, angle_old, speed_old
from arduino_serial import send_command, close
import logging

logger = logging.getLogger(__name__)

class RobotController:
    """Manages robot control."""
    def __init__(self):
        self.current_angle = current_angle
        self.current_speed = current_speed
        self.angle_old = angle_old
        self.speed_old = speed_old

    def control(self, steering_angle: float) -> None:
        angle_diff = abs(steering_angle - self.current_angle)

        self.current_speed = min(255, 100 + angle_diff)

        logger.info(f"Speed: {self.current_speed}, Angle diff: {angle_diff:.2f}")

        new_angle = int(steering_angle)
        if new_angle != self.angle_old:
            send_command(f'S{new_angle}\r\n'.encode())
            self.angle_old = new_angle

        if self.current_speed != self.speed_old:
            send_command(f'M{self.current_speed}\r\n'.encode())
            self.speed_old = self.current_speed

        self.current_angle = new_angle

    def close(self) -> None:
        send_command(b'STOP\r\n')
        close()
