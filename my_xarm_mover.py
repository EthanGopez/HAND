#!/usr/bin/env python3
import rclpy
from xarm_msgs.srv import PlanJoint, PlanExec


def main():
    rclpy.init()
    node = rclpy.create_node('my_xarm_mover')

    plan_client = node.create_client(PlanJoint, 'xarm_joint_plan')
    exec_client = node.create_client(PlanExec, 'xarm_exec_plan')
    plan_client.wait_for_service()
    exec_client.wait_for_service()

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result()

    targets = [
        [0.785, 0.0, 0.0, 0.0, 0.0, 0.0],      # base rotated 45°
        [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0],    # the "hold-up" pose
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],        # home
    ]

    for target in targets:
        planned = call(plan_client, PlanJoint.Request(target=target))
        if not planned.success:
            node.get_logger().error(f'No plan for {target}, stopping')
            break
        executed = call(exec_client, PlanExec.Request(wait=True))
        node.get_logger().info(f'Moved to {target}: {executed.success}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
