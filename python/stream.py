from flask import Flask, Response, request
import cv2
import config_city
import threading
import logging
import time

logger = logging.getLogger(__name__)

app = Flask(__name__)
server_thread = None

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

@app.route('/shutdown', methods=['POST'])
def shutdown():
    func = request.environ.get('werkzeug.server.shutdown')
    if func:
        func()
        return 'Server shutting down...'
    else:
        return 'Not running with the Werkzeug Server', 500

def start_stream():
    global server_thread
    server_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True, debug=False), daemon=True)
    server_thread.start()

def stop_stream():
    import requests
    try:
        requests.post("http://127.0.0.1:5000/shutdown")
        if server_thread:
            server_thread.join()
    except Exception as e:
        logger.error(f"Error shutting down stream: {e}")
