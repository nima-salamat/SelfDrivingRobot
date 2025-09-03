# Robot control variables
current_angle = 107      # Servo angle (30-150, 107 is straight)
current_speed = 200     # Speed (200-500, higher value = slower)
angle_old = 107
speed_old = 200
direction_old = b'STOP'

# Lane detection parameters
CANNY_LOW = 30
CANNY_HIGH = 130
HOUGH_THRESHOLD = 20    # Lowered for speed
MIN_LINE_LENGTH = 20
MAX_LINE_GAP = 20

# Frame size
F_Width = 384
F_Height = 216


# ROI for right lane detection (by percentage)
ROI_TOP_RL = 0.85  # 85% from top
ROI_BOTTOM_RL = 1.0  # 100% from top
ROI_LEFT_RL = 0.6   # 60% from left
ROI_RIGHT_RL = 1.0  # 100% from left

# ROI for left lane detection (by percentage)
ROI_TOP_LL = 0.85  # 85% from top
ROI_BOTTOM_LL = 1.0  # 100% from top
ROI_LEFT_LL = 0.0   # 0% from left
ROI_RIGHT_LL = 0.4  # 40% from left

# ROI for crosswalk detection (by percentage)
ROI_TOP_CW = 0.6    # 50% from top
ROI_BOTTOM_CW = 1 # 80% from top
ROI_LEFT_CW = 0.2   # 0% from left
ROI_RIGHT_CW = 0.8  # 100% from left

# ROI for AprilTag detection (by percentage)
ROI_TOP_AT = 0.0    # 0% from top
ROI_BOTTOM_AT = 1 # 30% from top
ROI_LEFT_AT = 0.0   # 0% from left
ROI_RIGHT_AT = 1.0  # 100% from left

# ROI for traffic light detection (by percentage)
ROI_TOP_TL = 0.0    # 0% from top
ROI_BOTTOM_TL = 0.2 # 20% from top
ROI_LEFT_TL = 0.35  # 25% from left
ROI_RIGHT_TL = 0.7 # 75% from left
