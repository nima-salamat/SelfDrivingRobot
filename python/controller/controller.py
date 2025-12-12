from utils.commands import ArduinoConnectionThreaded

class RobotController:
    def __init__(self):
        self.connection = ArduinoConnectionThreaded()
        self.current_angle = 90
        self.current_speed = 0

    def _send_command(self, cmd: str):
        cmd = cmd.strip() + "\n" 
        self.connection.send_command(cmd)

    def servo(self, angle: int):
        if angle < 30:
            angle = 30
        elif angle > 150:
            angle = 150
        
        self._send_command(f"servo {angle}")

    def motor(self, speed: int):
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255
            
        if self.current_speed != speed:
            self.current_speed = speed
        self._send_command(f"motor {speed}")

    def stop(self):
        """Stop the robot"""
        self._send_command("stop")
        self.current_speed = 0

    def set_angle(self, angle: int):
        self.servo(angle)
    
    def set_speed(self, speed: int):
        self.motor(speed)
    
    def forward(self, speed: int = None):
        if speed is None:
            speed = self.current_speed if self.current_speed > 0 else 150
        self.motor(abs(speed))
    
    def backward(self, speed: int = None):
        if speed is None:
            speed = self.current_speed if self.current_speed < 0 else -150
        self.motor(-abs(speed))
        
    def forward_pulse(self,s):
        self._send_command(s)
    
    def backward_pulse(self, s):
        self._send_command(s)
