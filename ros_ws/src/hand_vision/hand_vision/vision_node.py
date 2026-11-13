import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from hand_interfaces.msg import HandLandmarks


class VisionNode(Node):
    def __init__(self):
        super().__init__('hand_vision')
        self.source = self.declare_parameter('source', 'fake').value  # 'fake' or a camera URL
        rate_hz = self.declare_parameter('rate_hz', 30.0).value
        self.pub = self.create_publisher(HandLandmarks, 'hand/landmarks', 10)
        self.t0 = time.monotonic()
        self.camera = None
        if self.source != 'fake':
            self.camera = Camera(self.source, self.get_logger())
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f'Publishing /hand/landmarks from source={self.source}')

    def tick(self):
        msg = HandLandmarks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        points, handedness = self.camera.read() if self.camera else self.fake_hand()
        if points is not None:
            msg.tracked = True
            msg.handedness = handedness
            msg.landmarks = [Point(x=x, y=y, z=z) for x, y, z in points]
        self.pub.publish(msg)  # untracked frames are published too, with tracked=False

    def fake_hand(self):
        """A hand that circles the image centre for 8 s, then vanishes for 2 s."""
        t = time.monotonic() - self.t0
        if t % 10.0 >= 8.0:
            return None, ''
        cx = 0.5 + 0.15 * math.cos(0.5 * t)
        cy = 0.5 + 0.15 * math.sin(0.5 * t)
        return [(cx, cy + 0.01 * i, 0.0) for i in range(21)], 'Right'


class Camera:
    """Filled in at step 5. Until then, use source:=fake."""

    def __init__(self, url, logger):
        raise NotImplementedError('Real camera comes in step 5; run with source:=fake')


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