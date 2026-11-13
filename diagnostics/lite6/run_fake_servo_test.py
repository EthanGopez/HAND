"""Test real MoveIt Servo with fake hardware in a localhost-only ROS domain.

No physical driver, robot IP, GUI, or external command publisher is launched.
"""
import os
# Set these before importing ROS. Do not share the physical-arm graph.
os.environ['ROS_DOMAIN_ID'] = '218'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import importlib.util
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from check_kinematics import chain, evaluate
from hand_teleop.singularity_filter import SerialModel, SingularityFilter

DIRECTORY = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('fake_launch', DIRECTORY/'fake_servo.launch.py')
FAKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FAKE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guarded', action='store_true', help='Apply the same predictor as the mouse UI')
    args = parser.parse_args()
    prefix = 'fake_servo_filtered' if args.guarded else 'fake_servo'
    log_path = DIRECTORY/(prefix+'_test.log')
    results = []
    environment = dict(os.environ)
    process = None
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('offline_lite6_servo_probe')
        state, statuses, description = [None], [], [None]
        node.create_subscription(JointState, '/joint_states',
                                 lambda msg: state.__setitem__(0, msg), qos_profile_sensor_data)
        node.create_subscription(Int8, '/servo_server/status',
                                 lambda msg: statuses.append(msg.data), 100)
        node.create_subscription(
            String, '/robot_description', lambda msg: description.__setitem__(0, msg.data),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publish = node.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        services = {name: node.create_client(Trigger, '/servo_server/'+name)
                    for name in ('start_servo', 'stop_servo')}
        action = ActionClient(node, FollowJointTrajectory,
                              '/lite6_traj_controller/follow_joint_trajectory')

        def spin(seconds):
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=min(0.02, max(0,until-time.monotonic())))

        def wait(future, timeout=20):
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                raise RuntimeError('Fake Servo test service/action timed out')
            return future.result()

        def trigger(name):
            client = services[name]
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('Missing fake Servo service '+name)
            response = wait(client.call_async(Trigger.Request()))
            if not response.success:
                raise RuntimeError(response.message)

        def reset(degrees):
            trigger('stop_servo')
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [f'joint{i}' for i in range(1,7)]
            point = JointTrajectoryPoint()
            point.positions = [math.radians(v) for v in degrees]
            point.velocities = [0.0]*6
            point.time_from_start.sec = 1
            goal.trajectory.points = [point]
            handle = wait(action.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Fake-only reset goal rejected')
            result = wait(handle.get_result_async()).result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError('Fake-only reset failed: '+result.error_string)
            spin(0.2)
            current = dict(zip(state[0].name,state[0].position))
            assert max(abs(current[f'joint{i+1}']-math.radians(v))
                       for i,v in enumerate(degrees)) < 0.02
            trigger('start_servo')
            spin(0.2)

        model = chain(DIRECTORY/'model_snapshot.urdf')

        def joints():
            return {name: position for name,position in zip(state[0].name,state[0].position)
                    if name in {f'joint{i}' for i in range(1,7)}}

        def test(name, axis, speed, pose):
            reset(pose)
            statuses.clear()
            before = joints()
            transform, jacobian = evaluate(model,before)
            condition = float(np.linalg.cond(jacobian))
            allowed, blocked, reasons = 0, 0, set()
            started=time.monotonic()
            while time.monotonic()-started < 2.0:
                msg=TwistStamped();msg.header.frame_id='link_base'
                msg.header.stamp=node.get_clock().now().to_msg()
                velocity = [0.,0.,0.]
                velocity['xyz'.index(axis)] = speed
                if args.guarded:
                    decision = guard.filter(velocity, joints())
                    velocity = decision.velocity
                    if decision.blocked:
                        blocked += 1
                        reasons.add(decision.reason)
                    else:
                        allowed += 1
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
                publish.publish(msg)
                spin(0.05)
            stop=TwistStamped();stop.header.frame_id='link_base'
            stop.header.stamp=node.get_clock().now().to_msg();publish.publish(stop)
            spin(0.5)
            after=joints()
            end, end_jacobian=evaluate(model,after)
            row={'case':name,'speed_m_s':speed,'axis':axis,
                 'start_condition':condition,'end_condition':float(np.linalg.cond(end_jacobian)),
                 'servo_status_codes':sorted(set(statuses)),
                 'delta_xyz_m':(end[:3,3]-transform[:3,3]).tolist(),
                 'max_joint_change_degrees':max(abs(math.degrees(after[n]-before[n])) for n in before)}
            if args.guarded:
                row.update(allowed_samples=allowed, blocked_samples=blocked, blocked_reasons=sorted(reasons))
            results.append(row)
            print(json.dumps(row),flush=True)

        spin(0.5)
        other_nodes = set(node.get_node_names()) - {'offline_lite6_servo_probe'}
        if other_nodes:
            raise RuntimeError('Test domain 218 already in use: '+str(sorted(other_nodes)))

        with log_path.open('w') as logfile:
            process=subprocess.Popen(['ros2','launch',str(DIRECTORY/'fake_servo.launch.py')],
                stdout=logfile,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
            if not action.wait_for_server(timeout_sec=40):
                raise RuntimeError('Fake trajectory controller unavailable; see '+str(log_path))
            spin(0.3)
            assert state[0] is not None
            if args.guarded:
                spin(0.5)
                if description[0] is None:
                    raise RuntimeError('No fake robot description for the predictor')
                guard = SingularityFilter(SerialModel(description[0]))
            for axis in 'xyz':
                for sign in (1,-1):
                    test(('+' if sign>0 else '-')+axis.upper(),axis,sign*0.01,FAKE.POSE_DEGREES)
            test('all_zero_comparison','x',0.01,[0.0]*6)
        (DIRECTORY/(prefix+'_results.json')).write_text(json.dumps(results,indent=2)+'\n')
        if args.guarded:
            for row in results:
                stopped = row['case'] in ('+X', '+Y', '-Z', 'all_zero_comparison')
                if stopped:
                    assert row['blocked_samples'] > 0 and row['allowed_samples'] == 0, row
                    assert row['max_joint_change_degrees'] < 1e-6, row
                else:
                    assert row['allowed_samples'] > 0 and row['blocked_samples'] == 0, row
                    index = 'xyz'.index(row['axis'])
                    assert row['delta_xyz_m'][index]*row['speed_m_s'] > 1e-6, row
                assert not ({2, 4, 5} & set(row['servo_status_codes'])), row
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)


if __name__=='__main__':
    main()
