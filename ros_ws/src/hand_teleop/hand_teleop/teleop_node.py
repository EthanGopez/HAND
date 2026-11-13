import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from hand_interfaces.msg import HandLandmarks

WRIST = 0  # MediaPipe landmark index


class TeleopNode(Node):
    def __init__(self):
        super().__init__('hand_teleop')
        # Safety limits. Servo reads these as real units (m/s), so keep them small.
        self.max_speed = self.declare_parameter('max_speed', 0.05).value       # m/s hard cap
        self.gain = self.declare_parameter('gain', 0.25).value                 # m/s per unit of image offset
        self.deadband = self.declare_parameter('deadband', 0.05).value         # ignore offsets smaller than this
        self.stale_after = self.declare_parameter('stale_after', 0.2).value    # s without landmarks -> stop

        self.latest = None
        self.latest_time = None
        self.create_subscription(HandLandmarks, 'hand/landmarks', self.on_landmarks, 10)
        self.pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.create_timer(0.05, self.tick)  # 20 Hz, well inside Servo's 0.2 s command timeout
        self.get_logger().info(f'Teleop running, speed cap {self.max_speed} m/s')

    def on_landmarks(self, msg):
        self.latest = msg
        self.latest_time = self.get_clock().now()

    def tick(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'link_base'  # directions are relative to the arm's base

        if self.hand_is_fresh():
            wrist = self.latest.landmarks[WRIST]
            dx = wrist.x - 0.5  # + means the hand is right of the image centre
            dy = wrist.y - 0.5  # + means the hand is below the image centre
            cmd.twist.linear.y = self.limit(-dx)  # flip the sign if the motion comes out mirrored
            cmd.twist.linear.z = self.limit(-dy)  # hand up -> arm up
        # Otherwise every field stays 0: the arm holds still.
        self.pub.publish(cmd)

    def hand_is_fresh(self):
        if self.latest is None or not self.latest.tracked:
            return False
        age = (self.get_clock().now() - self.latest_time).nanoseconds * 1e-9
        return age < self.stale_after

    def limit(self, offset):
        if abs(offset) < self.deadband:
            return 0.0
        speed = self.gain * offset
        return max(-self.max_speed, min(self.max_speed, speed))


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
