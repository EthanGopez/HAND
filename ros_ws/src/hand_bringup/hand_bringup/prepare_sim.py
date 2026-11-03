"""Move a simulated arm out of its singular home pose before enabling teleop."""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rclpy.init()
    node = rclpy.create_node('prepare_sim')

    def wait(future, timeout=90.0):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Timed out waiting for simulation initialization')
        return future.result()

    def trigger(name):
        client = node.create_client(Trigger, '/servo_server/' + name)
        if not client.wait_for_service(timeout_sec=90.0):
            raise RuntimeError(f'Servo service unavailable: {name}')
        result = wait(client.call_async(Trigger.Request()))
        if not result.success:
            raise RuntimeError(f'{name} failed: {result.message}')

    try:
        # Servo must not publish a hold trajectory while the controller is
        # executing the ready move. Wait for completion rather than assuming
        # that a fixed startup delay guarantees a non-singular joint state.
        trigger('stop_servo')
        action = ActionClient(
            node, FollowJointTrajectory,
            '/xarm6_traj_controller/follow_joint_trajectory',
        )
        if not action.wait_for_server(timeout_sec=90.0):
            raise RuntimeError('Simulation trajectory controller unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.3, -0.5, 0.0, 0.8, 0.0]
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        handle = wait(action.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Simulation ready-pose goal rejected')
        result = wait(handle.get_result_async()).result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Ready move failed: {result.error_string}')
        trigger('start_servo')
        node.get_logger().info('Ready pose reached; enabling teleoperation')
    finally:
        node.destroy_node()
        rclpy.shutdown()
