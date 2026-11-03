"""
sweep: move the end effector from side to side, again and again, for a set time.

Handy for watching the arm in Gazebo. It alternates between two points either side of the
arm's centre line (+y and -y in link_base) at a fixed forward distance and height, keeping
the tool's current orientation, and goes through ee_target_tracker like any other goal.

    ros2 run hand_ee_target_tracker sweep                       # 15 minutes
    ros2 run hand_ee_target_tracker sweep --duration 600 --amplitude 0.2
"""
import argparse
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.utilities import remove_ros_args

from hand_ee_target_tracker.send_goal import GoalSender


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Sweep the end effector side to side through ee_target_tracker.')
    parser.add_argument('--duration', type=float, default=900.0,
                        help='how long to keep sweeping, seconds (default: 15 minutes)')
    parser.add_argument('--amplitude', type=float, default=0.15,
                        help='distance either side of the centre line, metres')
    parser.add_argument('--x', type=float, default=0.35, help='forward distance, metres')
    parser.add_argument('--z', type=float, default=0.25, help='height, metres')
    parser.add_argument('--dwell', type=float, default=0.5,
                        help='pause at each end, seconds')
    parser.add_argument('--rate', type=float, default=5.0, help='republish rate, Hz')
    parser.add_argument('--leg-timeout', type=float, default=60.0,
                        help='give up on one side after this many seconds')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])
    rclpy.init(args=argv)
    node = GoalSender()
    log = node.get_logger()
    code = 1
    try:
        orientation = node.current_orientation(wait=10.0)
        if orientation is None:
            log.error('No link_base -> link_eef transform; is the arm running?')
            return 1

        goal = PoseStamped()
        goal.header.frame_id = 'link_base'
        goal.pose.position.x = args.x
        goal.pose.position.z = args.z
        o = goal.pose.orientation
        o.x, o.y, o.z, o.w = orientation

        start = time.monotonic()
        side = 1.0
        legs = 0
        log.info(f'Sweeping y = +/-{args.amplitude} m at x = {args.x}, z = {args.z} '
                 f'for {args.duration:.0f} s')
        while rclpy.ok() and time.monotonic() - start < args.duration:
            goal.pose.position.y = side * args.amplitude
            outcome = node.go_to(goal, args.rate, args.leg_timeout, report=False)
            if outcome != 'reached':
                log.error(f'Gave up on y = {goal.pose.position.y:+.2f}: {outcome}')
                return 1
            legs += 1
            log.info(f'Leg {legs}: at y = {goal.pose.position.y:+.2f} m '
                     f'({time.monotonic() - start:.0f} s in)')
            side = -side
            pause_until = time.monotonic() + args.dwell
            while time.monotonic() < pause_until:
                goal.header.stamp = node.get_clock().now().to_msg()
                node.pub.publish(goal)  # keep the goal fresh so the arm holds there
                rclpy.spin_once(node, timeout_sec=1.0 / args.rate)
        log.info(f'Done after {legs} legs')
        code = 0
    except KeyboardInterrupt:
        log.info('Stopped')
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
