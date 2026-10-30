"""
send_goal: give the tracker a point and keep it fed until the arm gets there.

The tracker holds position once a goal is older than ``stale_after`` (1 s), so a single
``ros2 topic pub --once`` only moves the arm for a second. This tool republishes the
goal until the tracker reports REACHED.

    ros2 run hand_ee_target_tracker send_goal 0.3 0.0 0.3
    ros2 run hand_ee_target_tracker send_goal 0.3 0.0 0.3 --rpy 3.14159 0 0
"""
import argparse
import sys
import time

from geometry_msgs.msg import PoseStamped
from hand_interfaces.msg import EeTargetStatus
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from hand_ee_target_tracker import control

STATE_NAMES = {
    EeTargetStatus.NO_TARGET: 'NO_TARGET',
    EeTargetStatus.TRACKING: 'TRACKING',
    EeTargetStatus.REACHED: 'REACHED',
    EeTargetStatus.STALE: 'STALE',
    EeTargetStatus.REJECTED: 'REJECTED',
}
# A status that was published before our first goal arrived can still say REACHED.
SETTLE_TIME = 0.3  # s


def quat_from_rpy(roll, pitch, yaw):
    """Quaternion (x, y, z, w) for fixed-axis roll, pitch, yaw in radians."""
    qx = control.quat_from_rotation_vector((roll, 0.0, 0.0))
    qy = control.quat_from_rotation_vector((0.0, pitch, 0.0))
    qz = control.quat_from_rotation_vector((0.0, 0.0, yaw))
    return control.quat_multiply(qz, control.quat_multiply(qy, qx))


class GoalSender(Node):
    """Publishes one goal at a steady rate and watches the tracker's status."""

    def __init__(self):
        super().__init__('send_goal')
        self.status = None
        self.status_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, 'ee_target/raw', 10)
        self.create_subscription(EeTargetStatus, 'ee_target/status', self.on_status, 10)

    def on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def current_orientation(self, wait):
        """Orientation of link_eef in link_base, so a point goal does not rotate the wrist."""
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                r = self.tf_buffer.lookup_transform(
                    'link_base', 'link_eef', rclpy.time.Time()).transform.rotation
                return (r.x, r.y, r.z, r.w)
            except TransformException:
                continue
        return None


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Send an end-effector goal to ee_target_tracker and wait for it.')
    parser.add_argument('xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='goal position in the goal frame, metres')
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument('--quat', nargs=4, type=float, metavar=('X', 'Y', 'Z', 'W'),
                             help='goal orientation (default: keep the current one)')
    orientation.add_argument('--rpy', nargs=3, type=float, metavar=('R', 'P', 'Y'),
                             help='goal orientation as roll pitch yaw, radians')
    parser.add_argument('--frame', default='link_base', help='frame of the goal')
    parser.add_argument('--rate', type=float, default=5.0, help='republish rate, Hz')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='give up after this many seconds')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])
    rclpy.init(args=argv)
    node = GoalSender()
    code = 1
    try:
        if args.quat:
            orientation = control.normalize_quat(args.quat)
        elif args.rpy:
            orientation = quat_from_rpy(*args.rpy)
        else:
            orientation = node.current_orientation(wait=5.0)
            if orientation is None:
                node.get_logger().error(
                    'No link_base -> link_eef transform; is the arm running? '
                    'Pass --quat or --rpy to skip the lookup.')
                return 1
        if orientation is None:
            node.get_logger().error('That orientation is not a valid quaternion.')
            return 1

        goal = PoseStamped()
        goal.header.frame_id = args.frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = args.xyz
        o = goal.pose.orientation
        o.x, o.y, o.z, o.w = orientation

        start = time.monotonic()
        next_log = start
        period = 1.0 / args.rate
        while rclpy.ok() and time.monotonic() - start < args.timeout:
            goal.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(goal)
            end = time.monotonic() + period
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=max(0.0, end - time.monotonic()))
            status = node.status
            if status is None or time.monotonic() - start < SETTLE_TIME:
                continue
            if time.monotonic() >= next_log:
                node.get_logger().info(
                    f'{STATE_NAMES.get(status.state, status.state)}: '
                    f'{status.distance:.3f} m, {status.angle:.3f} rad to go')
                next_log += 1.0
            if status.state == EeTargetStatus.REACHED:
                node.get_logger().info(f'Reached in {time.monotonic() - start:.1f} s')
                code = 0
                break
            if status.state == EeTargetStatus.REJECTED:
                node.get_logger().error('The tracker rejected the goal (see its log)')
                break
        else:
            if code:
                node.get_logger().error(
                    'Timed out; is the tracker running with publish_twist:=true?'
                    if node.status is None else f'Timed out after {args.timeout:.0f} s')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
