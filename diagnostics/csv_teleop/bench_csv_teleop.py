#!/usr/bin/env python3
"""Exercise the CSV teleop pipeline with no MoveIt, no simulator, and no arm.

Stands in for the parts of the stack that are expensive to start: publishes a
latched robot description and steady joint feedback at a fixed pose, then
listens to what the two teleop nodes do with them. That is enough to satisfy
palm_teleop_node's guards, so the velocities it publishes are the ones it would
send to Servo.

Run the two nodes separately, then run this, for example:

    ros2 run hand_vision palm_csv_node --ros-args \\
        -p csv_path:=/workspace/palm_center.csv -p replay:=true
    ros2 run hand_teleop palm_teleop_node
    python3 diagnostics/csv_teleop/bench_csv_teleop.py --seconds 20

A pass here means the CSV reaches the arm command topic correctly. It says
nothing about Servo, the controller, or a physical robot.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hand_interfaces.msg import PalmCenter

DEFAULT_URDF = Path(__file__).resolve().parent.parent / 'lite6' / 'model_snapshot.urdf'
# Bent, away from the singular home pose, and inside the model's joint limits.
DEFAULT_POSE = [0.0, 0.5, 0.6, 0.0, 0.8, 0.0]
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--urdf', type=Path, default=DEFAULT_URDF,
                        help=f'Robot description to publish (default: {DEFAULT_URDF})')
    parser.add_argument('--pose', type=float, nargs=6, default=DEFAULT_POSE,
                        metavar='J', help='Six joint angles in radians to hold')
    parser.add_argument('--seconds', type=float, default=20.0,
                        help='How long to observe (default: 20)')
    parser.add_argument('--feedback-hz', type=float, default=50.0,
                        help='Joint feedback rate (default: 50)')
    parser.add_argument('--expect-motion', action='store_true',
                        help='Fail if no non-zero velocity is ever commanded')
    return parser.parse_args()


class Bench(Node):
    """Publish the robot's side of the conversation and record the teleop side."""

    def __init__(self, urdf_text, pose, feedback_hz):
        super().__init__('bench_csv_teleop')
        self.pose = list(pose)
        self.palms = []
        self.twists = []

        # Latched, so a node that starts later still receives it.
        description = self.create_publisher(String, '/robot_description', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        description.publish(String(data=urdf_text))
        self.description = description

        self.joints = self.create_publisher(JointState, '/joint_states', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0 / feedback_hz, self.publish_feedback)

        self.create_subscription(PalmCenter, '/hand/palm_center', self.on_palm, 10)
        self.create_subscription(TwistStamped, '/servo_server/delta_twist_cmds',
                                 self.on_twist, 10)

    def publish_feedback(self):
        msg = JointState()
        # The teleop guard rejects feedback older than 0.2 s, so every message
        # needs a current stamp even though the pose never changes.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.pose
        self.joints.publish(msg)

    def on_palm(self, msg):
        self.palms.append((time.monotonic(), msg.center.x, msg.center.y, msg.center.z))

    def on_twist(self, msg):
        self.twists.append((time.monotonic(), msg.twist.linear.x,
                            msg.twist.linear.y, msg.twist.linear.z,
                            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z))


def rate(samples) -> float:
    if len(samples) < 2:
        return 0.0
    span = samples[-1][0] - samples[0][0]
    return (len(samples) - 1) / span if span > 0 else 0.0


def report(bench, seconds, expect_motion) -> int:
    palms, twists = bench.palms, bench.twists
    speeds = [math.sqrt(t[1]**2 + t[2]**2 + t[3]**2) for t in twists]
    moving = [s for s in speeds if s > 1e-9]
    spin = max((max(abs(v) for v in t[4:]) for t in twists), default=0.0)

    print(f'\nobserved for {seconds:.0f} s')
    print(f'  /hand/palm_center            {len(palms):5d} msgs   {rate(palms):5.1f} Hz')
    print(f'  /servo_server/delta_twist…   {len(twists):5d} msgs   {rate(twists):5.1f} Hz')
    print(f'  non-zero velocity commands   {len(moving):5d}   '
          f'({100*len(moving)/len(twists):.0f}%)' if twists else '')
    if moving:
        print(f'  commanded speed              median {sorted(moving)[len(moving)//2]:.3f}'
              f'   max {max(moving):.3f} m/s')

    failures = []
    if not twists:
        failures.append('no velocity commands at all: is palm_teleop_node running?')
    else:
        if not 15.0 <= rate(twists) <= 25.0:
            failures.append(f'command rate {rate(twists):.1f} Hz is outside 15-25 Hz; '
                            "Servo's incoming_command_timeout is 0.2 s")
        if spin > 1e-9:
            failures.append(f'angular velocity is non-zero ({spin:.3f} rad/s); '
                            'this input should only translate the tool')
    if not palms:
        failures.append('no palm samples: is palm_csv_node running, and is the CSV live?')
    if expect_motion and not moving:
        failures.append('every command was zero: the guard blocked everything, or the '
                        'CSV produced no usable motion')

    print()
    for failure in failures:
        print(f'  FAIL  {failure}')
    if not failures:
        print('  PASS  CSV reaches the arm command topic at the expected rate.')
        print('        This does not exercise Servo, the controller, or any robot.')
    return 1 if failures else 0


def main() -> int:
    args = parse_args()
    if not args.urdf.is_file():
        raise SystemExit(f'Robot description not found: {args.urdf}')
    urdf_text = args.urdf.read_text()

    rclpy.init()
    bench = Bench(urdf_text, args.pose, args.feedback_hz)
    print(f'publishing /robot_description ({len(urdf_text)} bytes) and /joint_states '
          f'at {args.feedback_hz:g} Hz, pose {args.pose}')
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(bench, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    code = report(bench, args.seconds, args.expect_motion)
    bench.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
