import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ..modules.ultrasonic import Ultrasonic

ultrasonic = Ultrasonic()

while True:
    try:
        print(ultrasonic.ultrasonic_distance)
    except Exception:
        break