
from python.serial.connector import connect
from config import ANGLE_CENTER
arduino = connect()
speed = 200
arduino.send(f"command {ANGLE_CENTER} {speed}")

