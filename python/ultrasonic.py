from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import subprocess
import threading
import time

class Ultrasonic:
    def __init__(self, min_dist=0.3, echo=17, trigger=4, race=False, ser=None):
        factory = PiGPIOFactory()
        self.ultrasonic = DistanceSensor(echo=echo, trigger=trigger, pin_factory=factory)
        self.min_dist = min_dist
        self.ultrasonic_distance = 1.0
        self.race=race
        self.ser = ser
        self.ultrasonic_thread = threading.Thread(
            target=self._update_ultrasonic_distance, daemon=True
        )
        self.ultrasonic_thread.start()

    def active_pigpiod(self):
        subprocess.run(["sudo", "systemctl", "restart", "pigpiod"])
    
    
    def _update_ultrasonic_distance(self):
        """Continuously updates the ultrasonic distance measurement."""
        while True:
            self.ultrasonic_distance = self.ultrasonic.distance
            time.sleep(0.05)
    
    def handle_pass_block(self):
        """Executes a maneuver to pass a block."""
        self.ser.send("sharp left 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(1)

        self.ser.send("sharp right 160")
        time.sleep(0.8)

        self.ser.send("sharp left 160")
        time.sleep(0.8)
    
    def check_distance(self):
        # Check ultrasonic distance and react accordingly.
        if self.min_dist <= self.ultrasonic_distance <= self.min_dist + 0.05:
            if self.race:
                self.ser.send("center 0")
                time.sleep(0.5)
                if self.race:
                    self.handle_pass_block()
                    return True
            else:
                pass
        elif self.ultrasonic_distance < self.min_dist:
            if self.race:

                self.ser.send("center 0")
                time.sleep(0.3)
                while self.ultrasonic_distance <= self.min_dist + 0.1:
                    self.ser.send("center 0")
                    time.sleep(0.1)  
                self.handle_pass_block()
                return True    
            else:
                pass
    
        