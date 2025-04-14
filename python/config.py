import numpy as np

# ---------------- Adjustable Parameters ----------------
# ROI parameters (fractions of frame size)
ROI_Y_START_FRAC = 0.65  # Top of ROI (bottom 35% of frame)
ROI_X_START_FRAC = 0.5  # Left edge for lane detection
ROI_X_END_FRAC = 1  # Right edge for lane detection
ROI_WIDTH_FRAC = ROI_X_END_FRAC - ROI_X_START_FRAC

# Slope thresholds for lane lines
MIN_SLOPE = 0.7  # More lenient slope filter

# Hough transform parameters
HOUGH_RHO = 1
HOUGH_THETA = np.pi / 180
HOUGH_THRESHOLD = 25
HOUGH_MIN_LINE_LEN = 20
HOUGH_MAX_LINE_GAP = 10

# Canny edge detection thresholds
CANNY_LOW = 50
CANNY_HIGH = 150

# Gaussian blur kernel size (must be odd)
BLUR_KERNEL = 5

# Steering thresholds (in pixels)
THRESHOLD_SLOW = 20  # Slight deviation
THRESHOLD_MID = 45  # Moderate deviation
THRESHOLD_SHARP = 70  # Sharp turn threshold
THRESHOLD_VERY_SHARP = 110  # Sharp turn threshold

ANGLE_SHARP_LEFT = 20
ANGLE_MID_LEFT = 40
ANGLE_SLOW_LEFT = 55
ANGLE_CENTER = 70
ANGLE_SLOW_RIGHT = 85
ANGLE_MID_RIGHT = 100
ANGLE_SHARP_RIGHT = 120
ANGLE_VERY_SHARP_RIGHT = 140
ANGLE_VERY_SHARP_LEFT = 0


# Moving average history length for smoothing
HISTORY_LEN = 3

# Estimated lane width in pixels (for single-lane detection)
LANE_WIDTH_FRAC = 0.3

# Fixed center x-position for error calculation
FIXED_CENTER = 315

# Camera and production settings
CAMERA_INDEX = 0
TARGET_FPS = 30
FRAME_DELAY = 1 / TARGET_FPS

# Serial port settings
SERIAL_PORT = "/dev/ttyUSB0"  # Adjust as needed
BAUD_RATE = 9600

CAMERA_TOP_INDEX = 1  # ----------- attention!!
CAMERA_BOTTOM_INDEX = 0  # ----------- attention!!

CAMERA_WIDTH = 480
CAMERA_HEIGHT = 240


class MODE:
    """Speed Modes"""

    class Turbo:
        """Fast"""

        forward = 180
        turn = 140

    class Economy:
        """Medium Fast"""

        forward = 160
        turn = 130

    class Center:
        """Medium"""

        forward = 150
        turn = 130

    class Trot:
        """Medium Slow"""

        forward = 140
        turn = 120

    class Walk:
        """Slow"""

        forward = 130
        turn = 130

    class Snail:
        """Very Slow"""

        forward = 110
        turn = 100

    default = Walk

    @classmethod
    def set(cls, class_):
        cls.default = class_

    @classmethod
    def get(cls):
        return cls.default

    @classmethod
    def custom_mode(cls, forward, turn, set=True):
        if type(forward) is type(turn) is int:
            custom_speed = type("CustomSpeed", (), {"forward": forward, "turn": turn})
            if set:
                cls.default = custom_speed
            return cls.default

        else:
            raise ValueError("forward and turn must be <class:int>")


# -------------------------------------------------------

# Apriltag Variables
MIN_SIZE_APRILTAG = 10
TAG_LABLES = {
    0: "tunnel beginning",
    1: "tunnel end",
    2: "cross walk",
    3: "parking zone",
    4: "No-Passing Zone",
    5: "Passing Zone",
    6: "stop",
    7: "priority over",
    8: "Bared area",
    9: "step uphill",
    10: "step downhill",
    11: "turn left",
    12: "turn right",
    13: "go straight",
}

# main file variables
DEBUG = False
TRY_EXCEPT = False
# arduino parameter
ULTERASONIC_ENABLED = False

# apriltag config
THRESHOLD_VALUE = 100
USE_ADAPTIVE_THRESHOLD = True

# ROI_ORDER: "before" to crop the ROI before preprocessing,
#            "after" to crop after preprocessing.
ROI_ORDER = "after"  # or "before"

# DETECTOR_TYPE: "LSD" to use OpenCV's Line Segment Detector,
#                "HOUGH" to use HoughLinesP.
DETECTOR_TYPE = "HOUGH"  # or "LSD"

# -------------------------------------------------------
