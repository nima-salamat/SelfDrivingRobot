import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ..modules.serial_connector import connect

arduino = connect()
angle = 12
arduino.send(f"command {angle} 0")