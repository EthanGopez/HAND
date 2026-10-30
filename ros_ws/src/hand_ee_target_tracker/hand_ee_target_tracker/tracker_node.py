"""
ee_target_tracker: turn a slow end-effector goal into a stream Servo can follow.

See docs/interfaces/ee_target_tracker.md for the contract.
"""
import signal
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from hand_interfaces.msg import EeTargetStatus
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with Buffer.transform

from hand_ee_target_tracker import control

BASE_FRAME = 'link_base'  # Servo's robot_link_command_frame
EEF_FRAME = 'link_eef'    # Servo's ee_frame_name
TWIST_TOPIC = '/servo_server/delta_twist_cmds'
STATUS_PERIOD = 0.1       # s, 10 Hz
TF_MAX_AGE = 0.5          # s, older than this and the measured pose is not trusted


class EeTargetTracker(Node):
    """Holds the latest slow goal; every cycle emits a setpoint and a bounded twist."""

    def __init__(self):
        super().__init__('ee_target_tracker')
        self.limits = control.Limits(
            publish_rate=self.declare_parameter('publish_rate', 50.0).value,
            max_speed=self.declare_parameter('max_speed', 0.05).value,
            max_angular_speed=self.declare_parameter('max_angular_speed', 0.5).value,
            gain=self.declare_parameter('gain', 1.0).value,
            angular_gain=self.declare_parameter('angular_gain', 1.0).value,
            deadband=self.declare_parameter('deadband', 0.002).value,
            position_tolerance=self.declare_parameter('position_tolerance', 0.005).value,
            orientation_tolerance=self.declare_parameter('orientation_tolerance', 0.02).value,
        )
        self.stale_after = self.declare_parameter('stale_after', 1.0).value
        self.workspace_min = tuple(
            self.declare_parameter('workspace_min', [-0.7, -0.7, 0.0]).value)
        self.workspace_max = tuple(
            self.declare_parameter('workspace_max', [0.7, 0.7, 0.8]).value)
        publish_twist = self.declare_parameter('publish_twist', False).value
        self.validate_parameters()

        self.goal = None           # (position, unit quaternion) of the last accepted goal
        self.goal_time = None      # when the last goal arrived, whether or not it was accepted
        self.goal_rejected = False
        self.status = EeTargetStatus()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, 'ee_target/raw', self.on_goal, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, 'ee_target/setpoint', 10)
        self.status_pub = self.create_publisher(EeTargetStatus, 'ee_target/status', 10)
        # Only one node may drive Servo at a time (hand_teleop also publishes here),
        # so the publisher does not even exist unless this node was told to drive.
        self.twist_pub = None
        if publish_twist:
            self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        # The loop runs on the steady clock, not ROS time. In Gazebo ROS time is /clock, which
        # ticks at only 10 Hz, so a ROS-time timer would fire at most 10 times a second
        # and Servo would see a fraction of the commanded speed.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.limits.publish_rate, self.tick, clock=steady)
        self.create_timer(STATUS_PERIOD, self.publish_status, clock=steady)
        self.get_logger().info(
            f'ee_target_tracker running at {self.limits.publish_rate:g} Hz, speed cap '
            f'{self.limits.max_speed} m/s, driving Servo: {self.twist_pub is not None}')

    def validate_parameters(self):
        """Refuse to start with limits that would make the commands meaningless."""
        positive = ('publish_rate', 'max_speed', 'max_angular_speed')
        for name in positive:
            if getattr(self.limits, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if len(self.workspace_min) != 3 or len(self.workspace_max) != 3:
            raise ValueError('workspace_min and workspace_max need three values each')
        if any(lo > hi for lo, hi in zip(self.workspace_min, self.workspace_max)):
            raise ValueError('workspace_min must not exceed workspace_max')

    # -- goal in -----------------------------------------------------------------

    def on_goal(self, msg):
        self.goal_time = self.get_clock().now()
        self.goal = self.accept(msg)
        self.goal_rejected = self.goal is None

    def accept(self, msg):
        """Return (position, quaternion) in link_base, or None with the reason logged."""
        if not msg.header.frame_id:
            return self.reject('the goal has an empty frame_id')
        if msg.header.frame_id != BASE_FRAME:
            msg = self.to_base_frame(msg)
            if msg is None:
                return None
        p = msg.pose.position
        o = msg.pose.orientation
        position = (p.x, p.y, p.z)
        if not control.is_finite(position):
            return self.reject('the goal position is not finite')
        orientation = control.normalize_quat((o.x, o.y, o.z, o.w))
        if orientation is None:
            return self.reject('the goal orientation is not a valid quaternion '
                               '(all zeros?); set it to the current orientation to keep it')
        if not control.in_workspace(position, self.workspace_min, self.workspace_max):
            return self.reject(f'goal {tuple(round(c, 3) for c in position)} is outside '
                               f'the workspace box {self.workspace_min} to {self.workspace_max}')
        return position, orientation

    def to_base_frame(self, msg):
        latest = PoseStamped()
        latest.header.frame_id = msg.header.frame_id  # stamp 0 asks TF for the latest transform
        latest.pose = msg.pose
        try:
            return self.tf_buffer.transform(latest, BASE_FRAME, timeout=Duration())
        except TransformException as error:
            self.reject(f'no transform from {msg.header.frame_id} to {BASE_FRAME}: {error}')
            return None

    def reject(self, reason):
        self.get_logger().warn(f'Goal rejected: {reason}')
        return None

    # -- control cycle -----------------------------------------------------------

    def tick(self):
        now = self.get_clock().now()
        current = self.current_pose(now)
        if current is None:
            self.get_logger().warn('No fresh link_base -> link_eef transform; holding',
                                   throttle_duration_sec=2.0)
            self.send_twist(control.ZERO, control.ZERO, now)
            state = self.goal_state(now)
            if state == EeTargetStatus.TRACKING:  # cannot track without a measured pose
                state = EeTargetStatus.NO_TARGET
            self.set_status(state, 0.0, 0.0, now)
            return
        position, orientation = current

        state = self.goal_state(now)
        distance = angle = 0.0
        if state in (EeTargetStatus.TRACKING, EeTargetStatus.STALE):
            goal_position, goal_orientation = self.goal
            command = control.compute_command(position, orientation,
                                              goal_position, goal_orientation, self.limits)
            distance, angle = command.distance, command.angle
            if state == EeTargetStatus.STALE:
                command = control.hold_command(position, orientation)
            elif control.is_reached(distance, angle, self.limits):
                state = EeTargetStatus.REACHED
        else:
            command = control.hold_command(position, orientation)

        self.set_status(state, distance, angle, now)
        self.send_twist(command.linear, command.angular, now)
        self.send_setpoint(command, now)

    def goal_state(self, now):
        """State from the goal alone; TRACKING here becomes REACHED once errors are known."""
        if self.goal_rejected:
            return EeTargetStatus.REJECTED
        if self.goal is None:
            return EeTargetStatus.NO_TARGET
        if self.goal_age(now) > self.stale_after:
            return EeTargetStatus.STALE
        return EeTargetStatus.TRACKING

    def goal_age(self, now):
        return (now - self.goal_time).nanoseconds * 1e-9

    def current_pose(self, now):
        """Measured end-effector pose from TF, or None if it is missing or old."""
        try:
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, EEF_FRAME, Time())
        except TransformException:
            return None
        age = (now - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > TF_MAX_AGE:
            return None
        t = tf.transform.translation
        r = control.normalize_quat((tf.transform.rotation.x, tf.transform.rotation.y,
                                    tf.transform.rotation.z, tf.transform.rotation.w))
        if r is None:
            return None
        return (t.x, t.y, t.z), r

    # -- outputs -----------------------------------------------------------------

    def send_twist(self, linear, angular, now):
        if self.twist_pub is None:
            return
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = BASE_FRAME
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = linear
        cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z = angular
        self.twist_pub.publish(cmd)

    def send_setpoint(self, command, now):
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = BASE_FRAME
        p = msg.pose.position
        p.x, p.y, p.z = command.setpoint_position
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = command.setpoint_orientation
        self.setpoint_pub.publish(msg)

    def set_status(self, state, distance, angle, now):
        status = EeTargetStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = BASE_FRAME
        status.state = state
        status.distance = float(distance)
        status.angle = float(angle)
        has_goal = state in (EeTargetStatus.TRACKING, EeTargetStatus.REACHED,
                             EeTargetStatus.STALE)
        status.target_age = float(self.goal_age(now)) if has_goal else 0.0
        status.publishing_twist = self.twist_pub is not None
        self.status = status

    def publish_status(self):
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status.header.frame_id = BASE_FRAME
        self.status.publishing_twist = self.twist_pub is not None
        self.status_pub.publish(self.status)

    def stop(self):
        """Send one zero twist so Servo is not left holding the last velocity."""
        if self.twist_pub is not None and rclpy.ok():
            self.send_twist(control.ZERO, control.ZERO, self.get_clock().now())
            time.sleep(0.05)  # let the message leave before the context goes away


def main():
    # rclpy's own SIGINT handling shuts the context down before we could publish the
    # final zero twist, so take the signals ourselves.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def on_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node = EeTargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
