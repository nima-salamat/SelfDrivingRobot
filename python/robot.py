from config import *
from camera import PICamera
from control import RobotController
from detectors.lane_detector import VisionProcessor
import RPi.GPIO as GPIO
import time
from arduino_serial import send_command
import cv2
from typing import Dict
import logging
from collections import defaultdict
import threading

logger = logging.getLogger(__name__)


class LaneTrackingVehicle:
    global LAST_TAG
    """Main class for lane tracking and robot control (refactored to use VisionProcessor modes).

    Usage:
        vehicle = LaneTrackingVehicle(mode='race', display=True)
        vehicle.run()
    """

    def __init__(self, mode: str = "race", display: bool = True):
        self.config = {
            'F_Width': F_Width,
            'F_Height': F_Height,
            'CANNY_LOW': CANNY_LOW,
            'CANNY_HIGH': CANNY_HIGH,
            'HOUGH_THRESHOLD': HOUGH_THRESHOLD,
            'MIN_LINE_LENGTH': MIN_LINE_LENGTH,
            'MAX_LINE_GAP': MAX_LINE_GAP,
            'ROI_TOP_RL': ROI_TOP_RL,
            'ROI_BOTTOM_RL': ROI_BOTTOM_RL,
            'ROI_LEFT_RL': ROI_LEFT_RL,
            'ROI_RIGHT_RL': ROI_RIGHT_RL,
            'ROI_TOP_LL': ROI_TOP_LL,
            'ROI_BOTTOM_LL': ROI_BOTTOM_LL,
            'ROI_LEFT_LL': ROI_LEFT_LL,
            'ROI_RIGHT_LL': ROI_RIGHT_LL,
            'ROI_TOP_CW': ROI_TOP_CW,
            'ROI_BOTTOM_CW': ROI_BOTTOM_CW,
            'ROI_LEFT_CW': ROI_LEFT_CW,
            'ROI_RIGHT_CW': ROI_RIGHT_CW,
            'ROI_TOP_AT': ROI_TOP_AT,
            'ROI_BOTTOM_AT': ROI_BOTTOM_AT,
            'ROI_LEFT_AT': ROI_LEFT_AT,
            'ROI_RIGHT_AT': ROI_RIGHT_AT,
            'ROI_TOP_TL': ROI_TOP_TL,
            'ROI_BOTTOM_TL': ROI_BOTTOM_TL,
            'ROI_LEFT_TL': ROI_LEFT_TL,
            'ROI_RIGHT_TL': ROI_RIGHT_TL
        }

        # camera, robot, vision
        self.camera = PICamera(self.config['F_Width'], self.config['F_Height'])
        self.robot = RobotController()
        # Pass mode and debug/display flag to VisionProcessor
        self.vision = VisionProcessor(self.config, mode=mode, debug=display)

        self.mode = mode.lower()
        self.state = "WAITING"
        self.move_status = False
        self.gpio_pin = 23
        self.display_enabled = display
        self._setup_gpio()

    def _setup_gpio(self) -> None:
        """Initialize GPIO pin for start/stop control."""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            logger.info(f"GPIO {self.gpio_pin} initialized for start/stop")
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            self.gpio_pin = None

    def _check_gpio(self) -> bool:
        """Check GPIO pin state (1 for start, 0 for stop)."""
        if getattr(self, 'gpio_pin', None) is None:
            return False
        return GPIO.input(self.gpio_pin) == 1

    def run(self) -> None:
        global LAST_TAG

        cross_time = 0
        elapsed_time = 0

        while True:
            start_time = time.time()
            frame = self.camera.capture_frame()
            if frame is None:
                logger.warning("Failed to capture image")
                break

            # Use the unified detect method from VisionProcessor
            steering_angle, debug_frame, info = self.vision.detect(frame)

            # If vision.debug was False, debug_frame is None — fall back to original frame for display
            display_frame = debug_frame if debug_frame is not None else frame.copy()

            # Ensure info has defaults
            info = info or {}
            info.setdefault('cw_lines', 0)
            info.setdefault('april_tags_count', 0)
            info.setdefault('april_tags_ids', [])
            info.setdefault('tl_color', 'None')
            info.setdefault('line_center_x', frame.shape[1] // 2)
            info.setdefault('target_x', frame.shape[1] // 2)
            info.setdefault('error', 0.0)
            info.setdefault('lane_type', 'none')
            info.setdefault('speed', getattr(self.vision, 'current_speed', 100))
            info.setdefault('center_right_x', info['target_x'])
            info.setdefault('center_left_x', info['target_x'])

            # State machine
            if self.state == "WAITING":
                # show frame and wait for 'g' (or GPIO) to start
                self._display_frame(display_frame, info, 0)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('g') or self._check_gpio():
                    self.state = "LINE_FOLLOWING"
                    self.move_status = True
                    logger.info("Starting robot (G pressed or GPIO high)")
                elif key == ord('q'):
                    break

            elif self.state == "LINE_FOLLOWING":
                # Crosswalk handling (city mode only — but we can keep check for both)
                if info.get('cw_lines', 0) >= 7:
                    send_command(b'STOP\r\n')
                    if cross_time == 0:
                        cross_time = time.time()
                        logger.info("Crosswalk timer started")
                    elapsed_time = time.time() - cross_time

                    if elapsed_time >= 10:
                        logger.info("Crosswalk max-wait reached")

                    # After a short pause, check for Apriltags
                    if elapsed_time >= 3:
                        # Prefer IDs returned in info; fall back to global LAST_TAG
                        ids = info.get('april_tags_ids', [])
                        tag_id = ids[0] if ids else (LAST_TAG if 'LAST_TAG' in globals() else None)
                        logger.info("Crosswalk: checking APRILTAG id=%s", str(tag_id))

                        if tag_id == 12:
                            send_command(b'F\r\n')
                            logger.info("F sent (tag 12)")
                            time.sleep(1.5)
                            send_command(b'S140\r\n')
                            time.sleep(3)
                            send_command(b'STOP\r\n')

                        elif tag_id == 11:
                            send_command(b'F\r\n')
                            logger.info("F sent (tag 11)")
                            time.sleep(3.5)
                            send_command(b'S55\r\n')
                            time.sleep(2.3)
                            send_command(b'S90\r\n')
                            time.sleep(0.8)

                        elif tag_id == 13:
                            send_command(b'S90\r\n')
                            send_command(b'F\r\n')
                            logger.info("S90 + F (tag 13)")
                            time.sleep(2.5)

                        else:
                            # default behavior
                            send_command(b'S90\r\n')
                            send_command(b'F\r\n')
                            logger.info("Default AP-tag action")
                            time.sleep(2.5)

                        # Reset crosswalk timer and continue
                        cross_time = 0
                        elapsed_time = 0
                        self.state = "LINE_FOLLOWING"

                # Normal line following
                elif self.move_status:
                    send_command(b'F\r\n')
                    logger.debug("Command F sent for moving")
                    # Use robot controller to apply steering
                    try:
                        self.robot.control(steering_angle)
                    except Exception as e:
                        logger.exception("Robot control failed: %s", e)

                # Display frame and info
                end_time = time.time()
                fps = 1 / (end_time - start_time) if (end_time - start_time) > 0 else 0
                self._display_frame(display_frame, info, fps)

            # global key handling (stop / quit / manual commands)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('w') or not self._check_gpio():
                self.state = "LINE_FOLLOWING"
                self.move_status = True
                
                logger.info("Running robot (W pressed)")
            elif key == ord('q'):
                break
            elif key == ord('f'):
                send_command(b'F\r\n')
                logger.info("Manual F")
            elif key == ord('s'):
                send_command(b'STOP\r\n')

        self._cleanup()

    def _display_frame(self, frame: 'np.ndarray', line_info: Dict[str, float], fps: float) -> None:
        # Reuse original overlay logic but safely handle None
        if frame is None:
            return
        display_frame = frame.copy()
        height, width = display_frame.shape[:2]

        if line_info:
            line_center_x = line_info.get('line_center_x', width // 2)
            target_x = line_info.get('target_x', width // 2)
            error = line_info.get('error', 0.0)
            steering_angle = line_info.get('steering_angle', getattr(self.vision, 'last_steering_angle', 90))
            speed = line_info.get('speed', getattr(self.vision, 'current_speed', 100))
            lane_type = line_info.get('lane_type', 'none')
            center_right_x = line_info.get('center_right_x', target_x)
            center_left_x = line_info.get('center_left_x', target_x)
            cw_lines = line_info.get('cw_lines', 0)
            at_tags = line_info.get('april_tags_count', 0)
            tl_color = line_info.get('tl_color', 'None')

            cv2.line(display_frame, (int(line_center_x), 0), (int(line_center_x), height), (0, 0, 255), 2)
            cv2.line(display_frame, (int(target_x), 0), (int(target_x), height), (0, 255, 0), 2)

            cv2.putText(display_frame, f"Error: {error:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Steering: {steering_angle:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Speed: {speed:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"FPS: {fps:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Lane: {lane_type}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"R Center: {center_right_x:.2f}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"L Center: {center_left_x:.2f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"CW Lines: {cw_lines}", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"AT Tags: {at_tags}", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(display_frame, f"TL Color: {tl_color}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if self.display_enabled:
            cv2.imshow("Lane Tracking", display_frame)

    def _cleanup(self) -> None:
        try:
            self.camera.release()
        except Exception:
            pass
        try:
            self.robot.close()
        except Exception:
            pass
        try:
            GPIO.cleanup()
            logger.info("GPIO pins cleaned up")
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


class MultiTimeout:
    def __init__(self):
        self.timers = defaultdict(lambda: {'event': threading.Event(), 'start_time': 0})
        self.lock = threading.Lock()

    def start_timeout(self, seconds):
        with self.lock:
            timer_info = self.timers[seconds]
            if not timer_info['event'].is_set():
                return
            timer_info['event'].clear()
            timer_info['start_time'] = time.time()
            thread = threading.Thread(target=self._timeout_worker, args=(seconds, timer_info['event']))
            thread.daemon = True
            thread.start()

    def _timeout_worker(self, seconds, event):
        time.sleep(seconds)
        event.set()

    def is_timeout_finished(self, seconds):
        timer_info = self.timers[seconds]
        if timer_info['event'].is_set():
            return True
        elapsed = time.time() - timer_info['start_time']
        if elapsed >= seconds:
            timer_info['event'].set()
            return True
        return False


def start(mode: str = 'race', display: bool = True):
    timeout_manager = MultiTimeout()
    vehicle = LaneTrackingVehicle(mode=mode, display=display)
    vehicle.display_enabled = display
    try:
        vehicle.run()
    finally:
        send_command(b'STOP\r\n')
