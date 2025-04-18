import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ..modules.serial_connector import connect
from ..config import ANGLE_CENTER
arduino = connect()
speed = 200
arduino.send(f"command {ANGLE_CENTER} {speed}")

