"""
hand_vision: publish one HandLandmarks message per camera frame.

Tracking is gated by a tripwire (see tripwire.py). Until a palm enters the centre
zone every message has tracked=false, so hand_teleop keeps the arm still.

    ros2 run hand_vision vision_node
    ros2 run hand_vision vision_node --ros-args -p source:=0 -p mirror:=true
"""
import os
from pathlib import Path
import time

import cv2
from hand_interfaces.msg import HandLandmarks
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
import rclpy
from rclpy.node import Node

from hand_vision.message import build_message
from hand_vision.tripwire import TripwireGate

# camera stream served from the host, same as vision_demos/hand_tracking_demo.py
DEFAULT_SOURCE = 'http://host.docker.internal:5000/video'
DEFAULT_MODEL = str(Path.home() / '.cache' / 'mediapipe' / 'hand_landmarker.task')
MODEL_URL = (
    'https://storage.googleapis.com/mediapipe-models/hand_landmarker/'
    'hand_landmarker/float16/latest/hand_landmarker.task'
)


class VisionNode(Node):
    """Reads the camera, runs MediaPipe, and publishes tripwire-gated landmarks."""

    def __init__(self):
        super().__init__('hand_vision')
        source = str(self.declare_parameter('source', DEFAULT_SOURCE).value)
        model_path = self.declare_parameter('model_path', DEFAULT_MODEL).value
        self.mirror = self.declare_parameter('mirror', False).value
        rate = self.declare_parameter('rate', 30.0).value  # Hz, upper bound on frames read
        self.gate = TripwireGate(
            self.declare_parameter('tripwire_radius', 0.15).value,
            self.declare_parameter('lost_timeout', 5.0).value,
        )

        if not os.path.isfile(model_path):
            raise RuntimeError(
                f'MediaPipe model not found at {model_path}. Download it from {MODEL_URL} '
                'or run vision_demos/hand_tracking_demo.py once.')
        self.capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
        if not self.capture.isOpened():
            raise RuntimeError(f'Could not open video source: {source}')

        # one hand only: the message carries one hand and the arm follows one hand
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.last_timestamp_ms = -1

        self.pub = self.create_publisher(HandLandmarks, 'hand/landmarks', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'Reading {source}; waiting for a hand in the tripwire zone')

    def tick(self):
        ok, frame = self.capture.read()
        stamp = self.get_clock().now().to_msg()  # capture time of this frame
        if not ok or frame is None:
            self.get_logger().warn('No frame from camera', throttle_duration_sec=2.0)
            self.pub.publish(build_message(stamp))
            return
        if self.mirror:
            frame = cv2.flip(frame, 1)

        # MediaPipe VIDEO mode needs strictly increasing timestamps
        timestamp_ms = max(self.last_timestamp_ms + 1, time.monotonic_ns() // 1_000_000)
        self.last_timestamp_ms = timestamp_ms
        image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
        )
        result = self.landmarker.detect_for_video(image, timestamp_ms)

        height, width = frame.shape[:2]
        was_active = self.gate.active
        active = self.gate.update(result.hand_landmarks, width, height, time.monotonic())
        if active and not was_active:
            self.get_logger().info('Tripwire triggered: tracking started')
        elif was_active and not active:
            self.get_logger().info(
                f'No hand for {self.gate.lost_timeout_s:g} s: tracking stopped, '
                'waiting for tripwire')

        if active and result.hand_landmarks:
            msg = build_message(
                stamp,
                result.hand_landmarks[0],
                result.handedness[0][0].category_name,
            )
        else:
            msg = build_message(stamp)
        self.pub.publish(msg)

    def destroy_node(self):
        self.capture.release()
        self.landmarker.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
