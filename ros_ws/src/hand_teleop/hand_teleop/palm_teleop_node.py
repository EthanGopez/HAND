"""Drive MoveIt Servo from palm-centre samples, with the mouse node's guards.

This is the arm half of the CSV teleop pipeline. It subscribes to the palm
positions published by `hand_vision/palm_csv_node.py`, converts hand movement
into a Cartesian tool velocity through `PalmInput`, runs that velocity past
the same singularity and joint-feedback checks the mouse pad uses, and
publishes a `TwistStamped` to Servo at a fixed 20 Hz.

Servo, not this node, produces the joint trajectory. This node only ever says
"move the tool this fast in this direction, right now".
"""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from hand_interfaces.msg import PalmCenter

from hand_teleop.palm_input import PalmInput
from hand_teleop.servo_guard import ServoGuard

# 20 Hz keeps commands comfortably inside Servo's 0.2 s command timeout.
TICK_PERIOD = 0.05


class PalmTeleop(Node):
    """Publish stamped Cartesian velocities derived from hand movement."""

    def __init__(self):
        super().__init__('palm_teleop')
        max_speed = self.declare_parameter('max_speed', 0.05).value
        # Metres of tool travel per centimetre of CSV displacement. The CSV's
        # x, y and z are all centimetres, so 0.01 means "the tool tracks the
        # hand one-for-one". Depth (z) is noisier than x and y, so its scale
        # stays 0 (off) by default; raise it deliberately.
        scale_y = self.declare_parameter('scale_y', 0.01).value
        scale_z = self.declare_parameter('scale_z', 0.01).value
        scale_x = self.declare_parameter('scale_x', 0.0).value
        max_jump = self.declare_parameter('max_jump', 6.0).value
        max_depth_jump = self.declare_parameter('max_depth_jump', 1.0).value
        max_gap = self.declare_parameter('max_gap', 0.5).value
        deadband = self.declare_parameter('deadband', 0.0).value
        # No sample for this long means the hand is gone: stop, do not hold.
        self.stale_after = self.declare_parameter('stale_after', 0.2).value
        if not self.stale_after > 0:
            raise ValueError('stale_after must be greater than zero')

        self.controls = PalmInput(max_speed, scale_y=scale_y, scale_z=scale_z,
                                  scale_x=scale_x, max_jump=max_jump,
                                  max_depth_jump=max_depth_jump,
                                  max_gap=max_gap, deadband=deadband)
        description = self.declare_parameter('robot_description', '').value
        feedback_topic = self.declare_parameter('feedback_topic', '/joint_states').value
        self.guard = ServoGuard(self, description, feedback_topic,
                                on_stop=self.controls.stop)

        self.last_sample_wall = None
        self.last_tick = time.monotonic()
        self.samples = 0
        self.create_subscription(PalmCenter, 'hand/palm_center', self.on_palm, 10)
        self.pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.create_timer(TICK_PERIOD, self.tick)
        self.get_logger().info(
            f'Palm teleop ready, total speed cap {max_speed} m/s. '
            f'Scales: X {scale_x} (depth), Y {scale_y}, Z {scale_z}.'
        )

    def on_palm(self, msg):
        # The tracker's frame clock measures the motion; wall time only
        # measures how long ago we heard about it.
        self.controls.update(msg.center.x, msg.center.y, msg.center.z,
                             msg.source_timestamp_ms * 1e-3)
        self.last_sample_wall = time.monotonic()
        self.samples += 1

    def fresh(self, now):
        return (self.last_sample_wall is not None
                and 0 <= now - self.last_sample_wall <= self.stale_after)

    def publish(self, velocity):
        msg = TwistStamped()
        # Servo checks freshness, so every message needs a new stamp even
        # when the hand has not moved since the previous one.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
        # Angular velocity stays zero: the palm centre only translates the tool.
        self.pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if now - self.last_tick > 5 * TICK_PERIOD:
            # This process stalled. Disarm before publishing again, so a held
            # displacement cannot resume after Servo's own timeout stopped it.
            self.guard.invalidate()
        self.last_tick = now
        if not self.fresh(now):
            # No hand, or the vision node died. Either way: stop.
            self.guard.invalidate()
        decision = self.guard.decide(self.controls.velocity(), now)
        self.publish(decision.velocity)
        state = 'BLOCKED' if decision.blocked else (
            'ACTIVE' if self.controls.active else 'WAITING')
        self.get_logger().info(
            f'{state} | {decision.zone} | scale {decision.speed_scale:.2f} | '
            f'{decision.reason} | samples {self.samples} '
            f'rejected {self.controls.rejected} '
            f'(depth {self.controls.depth_rejected}) | '
            f'X {decision.velocity[0]:+.3f} Y {decision.velocity[1]:+.3f} '
            f'Z {decision.velocity[2]:+.3f} m/s',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PalmTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish((0.0, 0.0, 0.0))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
