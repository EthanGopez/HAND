"""Publish palm-centre positions read from the hand tracking demo's CSV.

This node is the vision half of the CSV teleop pipeline. It owns one job:
turn appended CSV rows into timestamped `hand_interfaces/msg/PalmCenter`
messages. It performs no mapping to robot axes and no rate limiting of the
arm; `hand_teleop/palm_teleop_node.py` does that, exactly as the mouse node
separates pointer events from the velocity it publishes to Servo.

Silence means "no hand". The demo writes rows only while its tripwire gate is
active, so this node publishes nothing when tracking is lost, and the teleop
node's staleness timeout stops the arm.
"""

import rclpy
from rclpy.node import Node

from hand_interfaces.msg import PalmCenter

from hand_vision.palm_csv_reader import PalmCsvReader


class PalmCsvPublisher(Node):
    """Follow a palm CSV and republish each new row."""

    def __init__(self):
        super().__init__('palm_csv_vision')
        path = self.declare_parameter('csv_path', 'palm_center.csv').value
        poll_hz = self.declare_parameter('poll_hz', 60.0).value
        # Live tracking must not replay the backlog; replay is for testing a
        # recorded run with no camera attached.
        replay = self.declare_parameter('replay', False).value
        replay_speed = self.declare_parameter('replay_speed', 1.0).value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        max_rows = self.declare_parameter('max_rows_per_tick', 240).value

        if not poll_hz > 0:
            raise ValueError('poll_hz must be greater than zero')
        if not replay_speed > 0:
            raise ValueError('replay_speed must be greater than zero')

        self.replay = bool(replay)
        self.replay_speed = float(replay_speed)
        self.reader = PalmCsvReader(path, from_start=self.replay,
                                    max_rows_per_read=int(max_rows))
        self.pending = []          # replay rows not yet due
        self.replay_origin = None  # (first source timestamp_ms, ROS time at start)
        self.published = 0
        self.last_status = ''

        self.pub = self.create_publisher(PalmCenter, 'hand/palm_center', 10)
        self.create_timer(1.0 / poll_hz, self.tick)
        mode = 'replaying' if self.replay else 'following'
        self.get_logger().info(
            f'Palm CSV vision node {mode} {path} -> /hand/palm_center '
            f'at {poll_hz:g} Hz polling'
        )

    def tick(self):
        samples = self.reader.read()
        if self.reader.status != self.last_status:
            # Report appearing, disappearing, and malformed files once each.
            self.last_status = self.reader.status
            self.get_logger().info(self.reader.status)
        if self.replay:
            self.pending.extend(samples)
            samples = self.due()
        for sample in samples:
            self.publish(sample)

    def due(self):
        """Release replayed rows no faster than they were recorded."""
        if not self.pending:
            return []
        now = self.get_clock().now()
        if self.replay_origin is None:
            self.replay_origin = (self.pending[0].timestamp_ms, now)
        first_ms, started = self.replay_origin
        elapsed_s = (now - started).nanoseconds * 1e-9 * self.replay_speed
        ready = []
        while self.pending and (self.pending[0].timestamp_ms - first_ms) * 1e-3 <= elapsed_s:
            ready.append(self.pending.pop(0))
        return ready

    def publish(self, sample):
        msg = PalmCenter()
        # The stamp is publication time, which is what a consumer needs for
        # freshness. The tracker's own clock travels in source_timestamp_ms,
        # because it is monotonic on the capture machine and not ROS time.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.source_timestamp_ms = sample.timestamp_ms
        msg.center.x = sample.x
        msg.center.y = sample.y
        msg.center.z = sample.z
        self.pub.publish(msg)
        self.published += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PalmCsvPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.reader.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
