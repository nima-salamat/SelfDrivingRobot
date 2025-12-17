
import logging
import sys
import time

logger = logging.getLogger(__name__)

try:
    from kivy.utils import platform as _kivy_platform
    platform = _kivy_platform
except Exception:
    platform = sys.platform

if platform == "android":
    from usb4a import usb
    from usbserial4a import serial4a
else:
    import serial

class ArduinoConnection:
    """
    Simple multi-platform USB/Serial connection.
    Only sends commands. No reading.
    """
    def __init__(self):
        self.serial_port = None

    def scan_devices(self):
        if platform == "android":
            devlist = usb.get_usb_device_list()
            return [d.getDeviceName() for d in devlist]
        else:
            from serial.tools import list_ports
            ports = list_ports.comports()
            return [p.device for p in ports]

    def open(self, device_name, baudrate=115200):
        if platform == "android":
            dev = usb.get_usb_device(device_name)
            if not dev:
                logger.error("Device not found: %s" % device_name)
                return False
            if not usb.has_usb_permission(dev):
                usb.request_usb_permission(dev)
                time.sleep(2)
            self.serial_port = serial4a.get_serial_port(
                device_name, baudrate, 8, 'N', 1, timeout=1
            )
        else:
            try:
                self.serial_port = serial.Serial(device_name, baudrate, timeout=1)
            except Exception as e:
                logger.error("Failed to open serial port: %s" % e)
                return False
        return True

    def send_command(self, command):
        if not self.serial_port:
            logger.error("Serial port not open")
            return False
        if isinstance(command, str):
            command = command.encode()
        try:
            self.serial_port.write(command)
            return True
        except Exception as e:
            logger.error("send_command failed: %s" % e)
            return False

    def close(self):
        try:
            if self.serial_port:
                self.serial_port.close()
        except:
            pass
        self.serial_port = None
