"""Offline checks: never execute a launch action or connect to a controller."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from sensor_msgs.msg import JointState

from hand_bringup.prepare_hardware import valid_feedback


LAUNCH_FILE = Path(__file__).parents[1] / 'launch' / 'hardware_mouse.launch.py'
spec = importlib.util.spec_from_file_location('hardware_mouse_launch', LAUNCH_FILE)
hardware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hardware)


def feedback():
    msg = JointState()
    msg.name = [f'joint{i}' for i in range(1, 7)]
    msg.position = [0.1] * 6
    msg.header.stamp.sec = 10
    return msg


@pytest.mark.parametrize('age,expected', [(0, True), (100_000_000, True),
                                         (600_000_000, False), (-1, False)])
def test_feedback_age(age, expected):
    assert valid_feedback(feedback(), 10_000_000_000 + age) is expected


@pytest.mark.parametrize('positions', [[0.1] * 5, [float('nan')] * 6,
                                     [float('inf')] * 6])
def test_feedback_rejects_incomplete_or_invalid_positions(positions):
    msg = feedback()
    msg.position = positions
    assert not valid_feedback(msg, 10_000_000_000)


def context(**overrides):
    result = LaunchContext()
    result.launch_configurations.update({
        'robot_ip': '192.0.2.10', 'max_speed': '0.05',
        'mouse_gain': '0.0001', 'show_rviz': 'false', **overrides,
    })
    return result


@pytest.mark.parametrize('overrides', [
    {'robot_ip': ''}, {'robot_ip': '127.0.0.1'}, {'robot_ip': '0.0.0.0'},
    {'robot_ip': '224.0.0.1'}, {'max_speed': 'nan'}, {'max_speed': '0'},
    {'mouse_gain': '-1'},
])
def test_invalid_settings_fail_before_hardware_actions(overrides):
    with patch.object(hardware, 'MoveItConfigsBuilder') as builder:
        with pytest.raises(ValueError):
            hardware.launch_setup(context(**overrides))
        builder.assert_not_called()


def test_real_stack_wiring_without_executing_actions():
    # Run real xacro/config generation, but never execute any resulting action.
    with patch.object(hardware, 'Node', wraps=hardware.Node) as nodes:
        actions = hardware.launch_setup(context())
    calls = [call.kwargs for call in nodes.call_args_list]
    assert actions
    assert not any(call['package'] in ('joy', 'gazebo_ros') for call in calls)
    assert not any(call['executable'] == 'prepare_sim' for call in calls)
    servo = next(call for call in calls if call['package'] == 'moveit_servo')
    description = servo['parameters'][0]['robot_description']
    assert 'uf_robot_hardware/UFRobotSystemHardware' in description
    assert '192.0.2.10' in description
    assert 'UFRobotFakeSystemHardware' not in description
    params = servo['parameters'][-1]
    assert params['use_sim_time'] is False
    assert params['moveit_servo']['move_group_name'] == 'lite6'
    assert params['moveit_servo']['command_out_topic'] == '/lite6_traj_controller/joint_trajectory'
    mouse = next(call for call in calls if call['executable'] == 'mouse_teleop_node')
    assert mouse['parameters'][0]['robot_description'] == description
    assert mouse['parameters'][1] == {'max_speed': 0.05, 'mouse_gain': 0.0001,
                                    'use_sim_time': False, 'feedback_topic': '/xarm/joint_states'}
    assert any(call['executable'] == 'prepare_hardware' for call in calls)
    assert any(call['executable'] == 'hardware_watchdog' for call in calls)



def test_xarm_only_discovery_cannot_select_hardware_for_lite6():
    with patch.object(hardware, 'MoveItConfigsBuilder') as builder:
        with pytest.raises(ValueError, match='explicit robot_ip'):
            hardware.launch_setup(context(robot_ip='auto'))
        builder.assert_not_called()
