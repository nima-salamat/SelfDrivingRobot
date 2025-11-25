from config import SPEED


class Navigate:
    def __init__(self, controller):
        self.controller = controller

    def left(self):
        self.controller.forward_pulse(f'f {SPEED} 7 90 f {SPEED} 5 40')
        
    def right(self):
        self.controller.forward_pulse(f'f {SPEED} 5 90 f {SPEED} 5 140')

    def straight(self):
        self.controller.forward_pulse(f'f {SPEED} 10 90')
