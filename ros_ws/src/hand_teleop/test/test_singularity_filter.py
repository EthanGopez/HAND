"""Pose-dependent command rejection using a real Lite6 URDF, no arm or GUI."""
import math
from pathlib import Path
from unittest.mock import MagicMock, patch
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from hand_teleop.mouse_teleop_node import MouseInput, MouseTeleop
from hand_teleop.singularity_filter import (
    SerialModel, SingularityFilter, fresh_feedback, joint_positions,
)

DESCRIPTION = (Path(__file__).parent/'fixtures/lite6_restricted.urdf').read_text()
POSE = [204.42974751901605, -11.143871979962466, 21.808630095310633,
        21.060790975538644, 32.95440599050424, -155.56641027220473]


@pytest.fixture
def simulated_filter():
    # This fixture changes J1 in a simulated model only; physical bounds stay restricted.
    robot = ET.fromstring(DESCRIPTION)
    limit = robot.find("joint[@name='joint1']/limit")
    limit.set('lower', str(-2*math.pi)); limit.set('upper', str(2*math.pi))
    return SingularityFilter(SerialModel(ET.tostring(robot, encoding='unicode')))


def positions(degrees=POSE):
    return {f'joint{i+1}': math.radians(v) for i,v in enumerate(degrees)}


@pytest.mark.parametrize('axis,sign,blocked', [
    (0,1,True), (0,-1,False), (1,1,True), (1,-1,False), (2,1,False), (2,-1,True),
])
def test_only_directions_approaching_singularity_are_blocked(simulated_filter, axis, sign, blocked):
    velocity = [0.,0.,0.]; velocity[axis] = sign*0.01
    decision = simulated_filter.filter(velocity, positions())
    assert decision.blocked is blocked
    assert decision.condition == pytest.approx(18.2002, abs=0.001)
    assert decision.velocity == ((0.,0.,0.) if blocked else tuple(velocity))


def test_same_mouse_direction_changes_with_robot_pose(simulated_filter):
    assert simulated_filter.filter((0.01,0,0), positions()).blocked
    changed = list(POSE); changed[0] -= 180
    assert not simulated_filter.filter((0.01,0,0), positions(changed)).blocked


def test_diagonal_is_rejected_whole_without_steering_in_another_direction(simulated_filter):
    decision = simulated_filter.filter((0.01,0.01,0), positions())
    assert decision.blocked
    assert decision.velocity == (0.,0.,0.)


def test_singular_pose_cannot_generate_nan_commands(simulated_filter):
    decision = simulated_filter.filter((0.01,0,0), positions([0]*6))
    assert decision.blocked
    assert decision.velocity == (0.,0.,0.)
    assert 'singularity' in decision.reason


def test_existing_physical_joint_ranges_are_never_widened():
    guard = SingularityFilter(SerialModel(DESCRIPTION))
    decision = guard.filter((0,0,0.01), positions())
    assert decision.blocked
    assert 'joint1' in decision.reason and 'configured joint limits' in decision.reason
    assert guard.model.upper[0] == pytest.approx(math.radians(178.2))


@pytest.mark.parametrize('velocity', [(float('nan'),0,0), (0,float('inf'),0), (0,0)])
def test_invalid_requests_fail_closed(simulated_filter, velocity):
    decision = simulated_filter.filter(velocity, positions())
    assert decision.blocked
    assert all(math.isfinite(v) and v == 0 for v in decision.velocity)


def test_missing_joint_values_fail_closed(simulated_filter):
    assert simulated_filter.filter((0.01,0,0), {}).blocked


def test_valid_stationary_input_is_zero(simulated_filter):
    decision = simulated_filter.filter((0,0,0), positions())
    assert not decision.blocked
    assert decision.velocity == (0.,0.,0.)


@pytest.mark.parametrize('stamp,received,ros_now,wall_now,expected', [
    (10_000_000_000, 1.0, 10_100_000_000, 1.1, True),
    (10_000_000_000, 1.0, 10_300_000_000, 1.1, False),
    (10_000_000_000, 1.0, 10_100_000_000, 1.3, False),
    (10_000_000_000, 1.0, 9_900_000_000, 1.1, False),
    (None, None, 10_000_000_000, 1.0, False),
])
def test_feedback_must_be_fresh_in_both_clocks(stamp, received, ros_now, wall_now, expected):
    assert fresh_feedback(stamp, received, ros_now, wall_now) is expected


@pytest.mark.parametrize('names,values', [
    (['joint1','joint1'], [0,0]), (['joint1'], []), (['joint1'], [float('nan')]),
])
def test_malformed_joint_messages_are_rejected(names, values):
    with pytest.raises(ValueError):
        joint_positions(names,values)


def test_jacobian_matches_independently_validated_condition(simulated_filter):
    model = simulated_filter.model
    q = np.array(list(positions().values()))
    jacobian = model.jacobian(q)
    # Check singular values against the previously validated independent FK implementation.
    assert np.linalg.cond(jacobian) == pytest.approx(18.200189734954037, rel=1e-7)
    assert np.linalg.matrix_rank(jacobian) == 6


def test_ui_tick_publishes_zero_for_blocked_drag_and_discards_it(simulated_filter):
    node = object.__new__(MouseTeleop)
    node.guard = simulated_filter; node.guard_error = ''
    node.positions = positions()
    node.joint_stamp_ns = 10_000_000_000; node.joint_received_wall = 0.0
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    node.controls = MouseInput(0.05,0.001)
    node.controls.start(0,0,0.0); node.controls.select_shift(True)
    node.controls.move(0,-0.5)  # Shift + drag up requests +X, blocked at this pose.
    node.last_tick = 0.0
    node.status = MagicMock(); node.guard_status = MagicMock()
    node.root = MagicMock(); node.publish = MagicMock()
    with patch('hand_teleop.mouse_teleop_node.rclpy.ok', return_value=True), \
            patch('hand_teleop.mouse_teleop_node.rclpy.spin_once'), \
            patch('hand_teleop.mouse_teleop_node.time.monotonic', return_value=0.05):
        node.tick()
    node.publish.assert_called_once_with((0.,0.,0.))
    assert 'BLOCKED' in node.status.set.call_args.args[0]
    assert 'singularity' in node.guard_status.set.call_args.args[0]
    # Held but stationary on the following tick cannot replay the discarded pixels.
    assert node.controls.velocity(0.1) == (0.,0.,0.)
    with patch('hand_teleop.mouse_teleop_node.rclpy.ok', return_value=True), \
            patch('hand_teleop.mouse_teleop_node.rclpy.spin_once'), \
            patch('hand_teleop.mouse_teleop_node.time.monotonic', return_value=0.15):
        node.tick()
    assert 'Last blocked input:' in node.guard_status.set.call_args.args[0]
    assert 'singularity' in node.guard_status.set.call_args.args[0]


def test_stale_pose_disarms_mouse_before_publishing(simulated_filter):
    node = object.__new__(MouseTeleop)
    node.guard = simulated_filter
    node.positions = positions()
    node.joint_stamp_ns = 10_000_000_000; node.joint_received_wall = 0.0
    node.get_clock = MagicMock()
    node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    node.controls = MouseInput(0.05)
    node.controls.start(0,0,0); node.controls.move(10,0)
    decision = node.filter_velocity((0,-0.01,0),1.0)
    assert decision.blocked and decision.velocity == (0.,0.,0.)
    assert not node.controls.active
