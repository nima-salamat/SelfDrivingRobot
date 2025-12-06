import logging
import cv2
import time
from flask import Flask, Response
import config_city
logger = logging.getLogger(__name__)


app = Flask(__name__)


@app.route('/')
def video_feed():
    def generate():
        while True:
            if config_city.debug_frame_buffer is None:
                time.sleep(0.01)
                continue
            ret, buffer = cv2.imencode('.jpg', config_city.debug_frame_buffer)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.01)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_stream():
    app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)

    

