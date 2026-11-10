import cv2
from flask import Flask, Response

# Smaller frames = less to encode, send and decode. MediaPipe shrinks every frame to
# ~200x200 internally, so more pixels only add latency. Keep 640x480: the depth
# constant in vision_demos/hand_tracking_demo.py is calibrated in pixels at this size.
WIDTH, HEIGHT = 640, 480
JPEG_QUALITY = 70  # OpenCV default is 95

app = Flask(__name__)
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
# some webcams ignore the request, so show what we actually got
print(f"Camera resolution: {camera.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
      f"{camera.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}, JPEG quality {JPEG_QUALITY}")

def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)