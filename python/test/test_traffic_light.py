import cv2
from vision.camera import Camera
from vision.traffic_light import TrafficLightDetector


camera = Camera()
tl_detector = TrafficLightDetector()
while True:
    try:
        frame = camera.capture_frame(resize=False)

        color, debug_frame = tl_detector.detect(frame)

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


