import serial
from config import (SERIAL_PORT, BAUD_RATE)



class Arduino:
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE, timeout=1):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
class ArduinoSerialConnector(Arduino):
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE, timeout=1):
        super().__init__(serial_port, baud_rate, timeout)
        self.initialize_serial()
    def initialize_serial(self):
        # Initialize serial connection
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            print(f"Serial port {SERIAL_PORT} opened at {BAUD_RATE} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
            
    
    def send(self, command:str):
        if self.ser is not None and self.ser.is_open:
            try:
                # print(f"Sending command to Arduino: {full_command}")
                self.ser.write((command + "\n").encode())
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
    def close(self):
        if self.ser is not None:
            self.ser.close()
def connect(*args):
    return ArduinoSerialConnector(*args)
