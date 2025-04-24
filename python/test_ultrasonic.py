
from ultrasonic import Ultrasonic

ultrasonic = Ultrasonic()

while True:
    try:
        print(ultrasonic.ultrasonic_distance)
    except Exception:
        break