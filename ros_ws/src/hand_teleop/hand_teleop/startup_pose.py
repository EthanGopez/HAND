"""Read-only startup assessment using measured feedback and the active model."""

import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hand_teleop.singularity_filter import (
    MotionDecision, SerialModel, SingularityFilter, fresh_feedback, joint_positions,
)


class StartupPose:
    def __init__(self, node, feedback_topic='/joint_states'):
        self.node = node
        self.filter = None
        self.positions = None
        self.stamp = self.received = None
        self.error = 'Waiting for robot model'
        node.create_subscription(
            String, '/robot_description', self.model,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(
            JointState, feedback_topic, self.feedback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def model(self, message):
        try:
            self.filter = SingularityFilter(SerialModel(message.data))
        except ValueError as error:
            self.filter = None
            self.error = str(error)

    def feedback(self, message):
        try:
            self.positions = joint_positions(message.name, message.position)
            self.stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.received = time.monotonic()
        except ValueError:
            self.positions = None

    def assess(self):
        if self.filter is None:
            return MotionDecision((0., 0., 0.), True, self.error)
        if self.positions is None or not fresh_feedback(
                self.stamp, self.received, self.node.get_clock().now().nanoseconds,
                time.monotonic()):
            return MotionDecision((0., 0., 0.), True, 'Waiting for fresh pose feedback')
        return self.filter.filter((0., 0., 0.), self.positions)
