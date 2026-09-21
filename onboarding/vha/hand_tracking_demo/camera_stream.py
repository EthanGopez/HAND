import cv2
from flask import Flask, Response

app = Flask(__name__)
camera = cv2.VideoCapture(0)

def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)