import serial
import sys
from config import current_angle, current_speed
import logging

logger = logging.getLogger(__name__)
class ArduinoSerial:
    serial_connection = None
    
    @classmethod
    def initialize(cls):
        # Initialize serial connection
        try:
            cls.serial_connection = serial.Serial('/dev/tty0', 115200)
            cls.send_command(b'STOP\r\n')
            cls.send_command(f'M{current_speed}\r\n'.encode())
            cls.send_command(f'S{current_angle}\r\n'.encode())
        except serial.SerialException:
            logger.error("Failed to initialize serial port. Robot control disabled.")
            cls.serial_connection = None
        

    @classmethod
    def send_command(cls, command: bytes) -> None:
        """Global function to send commands via serial."""
        if cls.serial_connection is not None and cls.serial_connection.is_open:
            cls.serial_connection.write(command)
        else:
            logger.warning(f"Serial connection not available for command: {command}")
            
    @classmethod
    def close(cls):
        if cls.serial_connection and cls.serial_connection.is_open:
            cls.serial_connection.close()
            

sys.modules[__name__] = ArduinoSerial