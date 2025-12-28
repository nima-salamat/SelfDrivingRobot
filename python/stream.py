import logging
import cv2
import time
from flask import Flask, Response, request
import threading
import base_config as temp_conf

if temp_conf.CONFIG_MODULE is not None:
    conf = temp_conf.CONFIG_MODULE
else:
    conf = temp_conf

logger = logging.getLogger(__name__)

stop_event = threading.Event()

app = Flask(__name__)

@app.route('/')
def video_feed():
    def generate():
        while not stop_event.is_set():
            if conf.debug_frame_buffer is None:
                time.sleep(0.01)
                continue
            ret, buffer = cv2.imencode('.jpg', conf.debug_frame_buffer)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.01)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/shutdown", methods=["POST"])
def shutdown_route():
    stop_event.set()
    func = request.environ.get('werkzeug.server.shutdown')
    if func:
        func()
    logger.debug("Server shutting down...")
    return "Server shutting down..."

def start_stream():
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
