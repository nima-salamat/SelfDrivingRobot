

from camera import UsbCamera
from config import FRAME_DELAY
import time
import cv2

indx_fist_cam = 0
indx_second_cam = 1

cam_1 = UsbCamera(indx_fist_cam)
cam_2 = UsbCamera(indx_second_cam)

while True:
    ret_cam_1, frame_cam_1 = cam_1.cap.read()
    ret_cam_2, frame_cam_2 = cam_2.cap.read()

    if not any([ret_cam_1, ret_cam_2]):
        print(
            f"Failed to read USB cam_1 frame index:{indx_fist_cam}" if not ret_cam_1 else "",
            f"Failed to read USB cam_2 frame index:{indx_second_cam}" if not ret_cam_2 else "",
            sep="\n",
        )
        break
        
    cv2.imshow(f"cam_1:{indx_fist_cam}", frame_cam_1)
    cv2.imshow(f"cam_2:{indx_second_cam}", frame_cam_1)

    if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    time.sleep(FRAME_DELAY)

cv2.destroyAllWindows()