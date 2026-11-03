"""Enable Servo after controller activation and fresh real-arm joint feedback.

Unlike prepare_sim, this node never sends a ready-pose trajectory.
"""

import math
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


JOINTS = {f'joint{i}' for i in range(1, 7)}
CONTROLLERS = {'joint_state_broadcaster', 'xarm6_traj_controller'}


def valid_feedback(msg, now_ns):
    """Require all six finite positions and a current wall-clock ROS stamp."""
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    positions = dict(zip(msg.name, msg.position))
    return (0 <= now_ns - stamp <= 500_000_000
            and JOINTS.issubset(positions)
            and all(math.isfinite(positions[name]) for name in JOINTS))


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('prepare_hardware')
    feedback = [None]
    subscription = node.create_subscription(
        JointState, '/joint_states', lambda msg: feedback.__setitem__(0, msg),
        qos_profile_sensor_data,
    )
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    start = node.create_client(Trigger, '/servo_server/start_servo')
    deadline = time.monotonic() + 90.0

    def fresh():
        return feedback[0] is not None and valid_feedback(
            feedback[0], node.get_clock().now().nanoseconds)

    def wait(future):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError('Timed out waiting for hardware readiness')
        return future.result()

    try:
        node.get_logger().info('Waiting for active controllers and fresh six-joint feedback')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not (fresh() and controllers.service_is_ready() and start.service_is_ready()):
                continue
            result = wait(controllers.call_async(ListControllers.Request()))
            active = {controller.name for controller in result.controller
                      if controller.state == 'active'}
            if not CONTROLLERS.issubset(active) or not fresh():
                continue
            result = wait(start.call_async(Trigger.Request()))
            if not result.success:
                raise RuntimeError(f'Servo start failed: {result.message}')
            if not fresh():
                raise RuntimeError('Joint feedback lost during Servo startup')
            node.get_logger().info('Hardware feedback ready; enabling mouse at current pose')
            return
        raise RuntimeError('Hardware readiness timed out; mouse input remains disabled')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
