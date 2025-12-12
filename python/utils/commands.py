import serial
import threading
import time
from serial import SerialException
from queue import Queue
serial_lock = threading.Lock()

class ArduinoConnection:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1, max_retries=3, reboot_wait=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_retries = max_retries
        self.reboot_wait = reboot_wait
        self.serial_connection = None
        self.init_serial_connection()

    def init_serial_connection(self, reopen=False):
        try:
            if reopen and self.serial_connection:
                try:
                    self.serial_connection.close()
                except:
                    pass
                time.sleep(0.2)

            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(self.reboot_wait)  # allow Arduino to reboot
            return True
        except SerialException:
            self.serial_connection = None
            return False

    def send_command(self, command):
        if isinstance(command, str):
            command = command.encode()

        for _ in range(self.max_retries):
            try:
                with serial_lock:
                    if not self.serial_connection or not self.serial_connection.is_open:
                        if not self.init_serial_connection(reopen=True):
                            time.sleep(0.1)
                            continue
                    self.serial_connection.write(command)
                    try:
                        self.serial_connection.flush()
                    except:
                        pass
            except Exception:
                # reopen and retry
                try:
                    self.init_serial_connection(reopen=True)
                except:
                    pass
                time.sleep(0.1)

    def close(self):
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass

class ArduinoConnectionThreaded(ArduinoConnection):
    def __init__(self):
        super().__init__()
        self.command_queue = Queue()
        self.thread = threading.Thread(target=self.sender, args=(self.command_queue,))
        self.thread.start()

    def send_command(self, command):
        self.command_queue.put(command)
        
    def sender(self, command_queue):
        while True:
            if not command_queue.empty():
                command = command_queue.get()
                super().send_command(command)