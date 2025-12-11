import cv2
from vision.camera import Camera
from vision.apriltag import ApriltagDetector 


camera = Camera()
at_detector = ApriltagDetector()
while True:
    try:
        frame = camera.capture_frame(resize=False)

        tags, debug_frame, largest_tag = at_detector.detect(frame)

        if debug_frame is not None:
            cv2.imshow("debug", debug_frame)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)
camera.release()
cv2.destroyAllWindows()


