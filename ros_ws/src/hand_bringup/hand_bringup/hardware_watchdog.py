"""Exit the hardware launch when successful driver feedback stops arriving."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from hand_bringup.prepare_hardware import valid_feedback


class FeedbackHealth:
    """Latch bad, stale, or missing feedback; restoration never resumes control."""

    def __init__(self, started, timeout=0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Feedback timeout must be finite and positive')
        self.started = started
        self.timeout = timeout
        self.last_received = None
        self.last_stamp_ns = None
        self.failure = None

    def observe(self, message, wall_now, ros_now_ns):
        if self.failure is not None:
            return
        if not valid_feedback(message, ros_now_ns):
            self.failure = 'Invalid or stale successful-read feedback'
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        # Replayed identical samples cannot keep the watchdog alive.
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns
        self.last_received = wall_now

    def check(self, wall_now, ros_now_ns):
        if self.failure is not None:
            return self.failure
        previous = self.started if self.last_received is None else self.last_received
        if wall_now - previous > self.timeout:
            self.failure = 'Successful joint feedback stopped; hardware launch must restart'
        elif self.last_stamp_ns is not None and not 0 <= ros_now_ns-self.last_stamp_ns <= int(self.timeout*1e9):
            self.failure = 'Joint feedback clock is stale or inconsistent'
        return self.failure


class HardwareWatchdog(Node):
    def __init__(self):
        super().__init__('hardware_watchdog')
        self.health = FeedbackHealth(time.monotonic())
        # The patched driver publishes this only after successful SDK reads.
        self.subscription = self.create_subscription(
            JointState, '/xarm/joint_states', self.observe, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.check)

    def observe(self, message):
        self.health.observe(message, time.monotonic(), self.get_clock().now().nanoseconds)

    def check(self):
        failure = self.health.check(time.monotonic(), self.get_clock().now().nanoseconds)
        if failure:
            raise RuntimeError(failure)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareWatchdog()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if node is not None:
            node.get_logger().error(str(error))
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
