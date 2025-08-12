
from python.serial.connector import connect

arduino = connect()
angle = 12
arduino.send(f"command {angle} 0")