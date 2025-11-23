from config import SPEED
class Navigate:
    def __init__(self, controller):
        self.controller = controller

    def left(self):
        self.controller.forward_pulse(SPEED, 5, 90)
        self.controller.forward_pulse(SPEED, 3, 40)
        self.controller.forward_pulse(SPEED, 2, 90)
        
        
        
    def right(self):
        self.controller.forward_pulse(SPEED, 3, 90)
        self.controller.forward_pulse(SPEED, 2, 130)
        self.controller.forward_pulse(SPEED, 1, 90)
        
    
    def straight(self):
        self.controller.forward_pulse(SPEED, 7, 90)
