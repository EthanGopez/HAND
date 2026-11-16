"""Wrap the mouse node's robot-model and joint-feedback checks for reuse.

`mouse_teleop_node.py` refuses to publish a velocity unless three things hold:
a usable robot model has been received, joint feedback is fresh, and the
singularity filter allows the requested direction. Those checks are not
specific to a mouse, so they live here and any input node can apply them.

The mouse node keeps its own copy for now; this module is written so it can
adopt it without changing behaviour.
"""

import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hand_teleop.singularity_filter import (
    MotionDecision, SerialModel, SingularityFilter, fresh_feedback, joint_positions,
)


class ServoGuard:
    """Decide whether a requested Cartesian velocity may reach Servo.

    `on_stop` is called whenever the guard invalidates the input source, so a
    stale pose cannot leave an input accumulating motion it will later replay.
    """

    def __init__(self, node, description='', feedback_topic='/joint_states',
                 on_stop=None):
        self.node = node
        self.on_stop = on_stop or (lambda: None)
        self.filter = None
        self.error = 'Waiting for robot model'
        self.positions = None
        self.stamp_ns = None
        self.received_wall = None
        node.create_subscription(
            JointState, feedback_topic, self.joint_feedback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        if description:
            self.load_model(description)
        else:
            # The description is latched, so a late subscriber still gets it.
            node.create_subscription(
                String, '/robot_description', lambda msg: self.load_model(msg.data),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

    @property
    def ready(self):
        return self.filter is not None

    def load_model(self, description):
        try:
            motion_filter = SingularityFilter(SerialModel(description))
        except ValueError as error:
            self.filter = None
            self.error = str(error)
            return
        self.filter = motion_filter
        self.error = ''
        # A changed model cannot replay input gathered against the old one.
        self.on_stop()

    def joint_feedback(self, message):
        try:
            self.positions = joint_positions(message.name, message.position)
            self.stamp_ns = (message.header.stamp.sec * 1_000_000_000
                             + message.header.stamp.nanosec)
            self.received_wall = time.monotonic()
        except ValueError:
            self.positions = None
            self.on_stop()

    def decide(self, velocity, wall_now=None):
        """Return the velocity that may be published, and why."""
        wall_now = time.monotonic() if wall_now is None else wall_now
        if self.filter is None:
            return MotionDecision((0.0, 0.0, 0.0), True, self.error)
        if self.positions is None or not fresh_feedback(
                self.stamp_ns, self.received_wall,
                self.node.get_clock().now().nanoseconds, wall_now):
            self.on_stop()
            return MotionDecision((0.0, 0.0, 0.0), True,
                                  'Waiting for fresh joint feedback')
        return self.filter.filter(velocity, self.positions)
