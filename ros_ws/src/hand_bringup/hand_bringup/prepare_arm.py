"""prepare_arm: park the simulated xArm6 where Servo can work from, then start Servo.

Two things have to happen, in this order, before Servo will follow commands:

1. The arm must leave its all-zeros start pose. That is a wrist singularity, so Servo
   halts ("Very close to a singularity, emergency stop") and ignores every twist.
2. Servo must be started with its start_servo service. Humble does not start it on launch.

The move comes first because a started Servo publishes hold trajectories to the same
controller, which override any other command until the arm is moved away from a
singularity.
"""
import sys

from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

CONTROLLER = '/xarm6_traj_controller'
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# End effector near (0.32, 0.0, 0.33) m in link_base, tool pointing straight down, and
# well away from both the wrist singularity at all zeros and the reach limits. Joints 2,
# 3 and 5 are parallel pitch axes, so they sum to zero to keep the tool vertical.
READY = [0.0, -0.5, -0.5, 0.0, 1.0, 0.0]
WAIT = 120.0  # s, for the simulator to bring the controller up


class ArmPreparer(Node):
    """Moves the arm to the ready pose and starts Servo."""

    def __init__(self):
        super().__init__('prepare_arm')
        self.move_time = self.declare_parameter('move_time', 4.0).value
        self.ready = list(self.declare_parameter('ready_pose', READY).value)
        self.action = ActionClient(self, FollowJointTrajectory,
                                   f'{CONTROLLER}/follow_joint_trajectory')
        self.start_servo = self.create_client(Trigger, '/servo_server/start_servo')

    def move_to_ready(self):
        if not self.action.wait_for_server(timeout_sec=WAIT):
            self.get_logger().error(f'{CONTROLLER} did not come up; is Gazebo running?')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint(positions=self.ready)
        seconds = int(self.move_time)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int((self.move_time - seconds) * 1e9))
        goal.trajectory.points = [point]
        self.get_logger().info(f'Moving to the ready pose {self.ready}')
        handle_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle_future)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('The controller rejected the ready-pose goal')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'Move to the ready pose failed (error_code {code})')
            return False
        return True

    def start(self):
        if not self.start_servo.wait_for_service(timeout_sec=WAIT):
            self.get_logger().error('/servo_server/start_servo is not available')
            return False
        future = self.start_servo.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error('Servo refused to start')
            return False
        self.get_logger().info('Servo started; the arm is ready for commands')
        return True


def main(argv=None):
    rclpy.init(args=argv)
    node = ArmPreparer()
    ok = False
    try:
        ok = node.move_to_ready() and node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
