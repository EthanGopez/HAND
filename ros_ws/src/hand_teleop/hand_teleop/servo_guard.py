"""Wrap the mouse node's robot-model and joint-feedback checks for reuse.

`mouse_teleop_node.py` refuses to publish a velocity unless three things hold:
a usable robot model has been received, joint feedback is fresh, and the
singularity filter allows the requested direction. Those checks are not
specific to a mouse, so they live here and any input node can apply them.

Mouse and palm input share this policy and diagnostic output.
"""

import time
import json
import math
from dataclasses import asdict

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from hand_teleop.recovery_hold import RecoveryHold

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
        self.allow_recovery = node.declare_parameter('allow_recovery', False).value
        if not isinstance(self.allow_recovery, bool):
            raise ValueError('allow_recovery must be boolean')
        self.hold = RecoveryHold()
        self.previous_zone = None
        node.create_subscription(Bool, '~/recovery_hold', self.recovery_hold, 1)
        self.filter = None
        self.error = 'Waiting for robot model'
        self.positions = None
        self.stamp_ns = None
        self.received_wall = None
        self.settings = {
            name: node.declare_parameter('singularity_' + name, default).value
            for name, default in [('warning', 17.0), ('halt', 25.0),
                                  ('horizon', 0.3), ('step', 0.05), ('hysteresis', 0.5),
                                  ('recovery_speed', 0.005), ('recovery_ceiling', 28.0)]
        }
        self.status = node.create_publisher(String, '~/guard_status', 10)
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

    def recovery_hold(self, message):
        self.hold.update(message.data, time.monotonic())
        if not message.data:
            self.on_stop()

    def invalidate(self):
        self.hold.reset()
        self.on_stop()

    def load_model(self, description):
        try:
            motion_filter = SingularityFilter(SerialModel(description), **self.settings)
        except ValueError as error:
            self.filter = None
            self.error = str(error)
            self.invalidate()
            return
        self.filter = motion_filter
        self.error = ''
        # A changed model cannot replay input gathered against the old one.
        self.invalidate()

    def joint_feedback(self, message):
        try:
            self.positions = joint_positions(message.name, message.position)
            self.stamp_ns = (message.header.stamp.sec * 1_000_000_000
                             + message.header.stamp.nanosec)
            self.received_wall = time.monotonic()
        except ValueError:
            self.positions = None
            if self.filter is not None:
                self.filter.reset()
            self.invalidate()

    def decide(self, velocity, wall_now=None):
        decision = self._decide(velocity, wall_now)
        if decision.zone != self.previous_zone:
            if 'RECOVERY' in (decision.zone, self.previous_zone):
                self.invalidate()
                decision = MotionDecision((0., 0., 0.), True,
                                          'Zone changed; release and re-establish input',
                                          decision.condition, decision.zone, 0.,
                                          decision.predicted_condition)
            self.previous_zone = decision.zone
        fields = asdict(decision)
        for name in ('condition', 'predicted_condition'):
            if fields[name] is not None and not math.isfinite(fields[name]):
                fields[name] = None
        self.status.publish(String(data=json.dumps(fields, allow_nan=False)))
        return decision

    def _decide(self, velocity, wall_now=None):
        """Return the velocity that may be published, and why."""
        wall_now = time.monotonic() if wall_now is None else wall_now
        if self.filter is None:
            self.invalidate()
            return MotionDecision((0.0, 0.0, 0.0), True, self.error)
        if self.positions is None or not fresh_feedback(
                self.stamp_ns, self.received_wall,
                self.node.get_clock().now().nanoseconds, wall_now):
            self.invalidate()
            self.filter.reset()
            return MotionDecision((0.0, 0.0, 0.0), True,
                                  'Waiting for fresh joint feedback')
        recovery_enabled = self.allow_recovery and self.hold.active(wall_now)
        decision = self.filter.filter(velocity, self.positions,
                                      recovery_enabled=recovery_enabled)
        if decision.blocked and (decision.zone != 'RECOVERY' or recovery_enabled):
            self.invalidate()
        return decision
