import math

import pytest

from hand_ee_target_tracker import control
from hand_ee_target_tracker.control import Limits

IDENTITY = (0.0, 0.0, 0.0, 1.0)
LIMITS = Limits()


def about(value, expected, tol=1e-9):
    return value == pytest.approx(expected, abs=tol)


def rot_z(angle):
    return control.quat_from_rotation_vector((0.0, 0.0, angle))


def command(goal, current=(0.0, 0.0, 0.0), limits=LIMITS,
            current_q=IDENTITY, goal_q=IDENTITY):
    return control.compute_command(current, current_q, goal, goal_q, limits)


def test_speed_is_capped_far_from_the_goal():
    cmd = command((1.0, 0.0, 0.0))
    assert about(cmd.linear[0], LIMITS.max_speed)
    assert about(cmd.linear[1], 0.0) and about(cmd.linear[2], 0.0)


def test_speed_is_proportional_near_the_goal():
    cmd = command((0.02, 0.0, 0.0))  # 0.02 m * gain 1.0 = 0.02 m/s, below the cap
    assert about(cmd.linear[0], 0.02)


def test_direction_points_at_the_goal():
    cmd = command((0.3, 0.4, 0.0), current=(0.0, 0.0, 0.0))
    speed = control.norm(cmd.linear)
    assert about(speed, LIMITS.max_speed)
    assert about(cmd.linear[0] / speed, 0.6)
    assert about(cmd.linear[1] / speed, 0.8)
    assert about(cmd.distance, 0.5)


def test_diagonal_speed_is_capped_on_the_total_not_per_axis():
    cmd = command((1.0, 1.0, 1.0))
    assert about(control.norm(cmd.linear), LIMITS.max_speed)


def test_deadband_is_on_total_error_not_per_axis():
    inside = command((0.0015, 0.0015, 0.0))   # |error| = 0.0021 is over the 0.002 deadband
    assert control.norm(inside.linear) > 0.0
    below = command((0.001, 0.001, 0.0))      # |error| = 0.0014 is inside it
    assert below.linear == control.ZERO


def test_goal_exactly_at_the_current_pose_gives_zero_twist():
    cmd = command((0.1, 0.2, 0.3), current=(0.1, 0.2, 0.3))
    assert cmd.linear == control.ZERO and cmd.angular == control.ZERO
    assert about(cmd.distance, 0.0) and about(cmd.angle, 0.0)


def test_setpoint_is_current_pose_advanced_by_one_cycle():
    cmd = command((1.0, 0.2, 0.3), current=(0.1, 0.2, 0.3))  # error is along +x only
    dt = 1.0 / LIMITS.publish_rate
    assert about(cmd.setpoint_position[0], 0.1 + LIMITS.max_speed * dt)
    assert about(cmd.setpoint_position[1], 0.2)
    assert about(cmd.setpoint_position[2], 0.3)


def test_setpoint_never_overshoots_by_construction():
    cmd = command((0.003, 0.0, 0.0))  # gain 1.0 gives 0.003 m/s, a 6e-5 m step at 50 Hz
    assert cmd.setpoint_position[0] < 0.003


def test_angular_speed_is_capped():
    cmd = command((0.0, 0.0, 0.0), goal_q=rot_z(2.0))   # 2 rad error, gain 1.0, cap 0.5
    assert about(control.norm(cmd.angular), LIMITS.max_angular_speed)
    assert about(cmd.angular[2] / control.norm(cmd.angular), 1.0)
    assert about(cmd.angle, 2.0, tol=1e-9)


def test_angular_speed_is_proportional_near_the_goal():
    cmd = command((0.0, 0.0, 0.0), goal_q=rot_z(0.1))
    assert about(cmd.angular[2], 0.1)


def test_rotation_takes_the_shortest_way_round():
    cmd = command((0.0, 0.0, 0.0), goal_q=rot_z(math.radians(350)))
    assert cmd.angular[2] < 0.0
    assert about(cmd.angle, math.radians(10), tol=1e-9)
    negated = tuple(-c for c in rot_z(math.radians(20)))   # same rotation, opposite sign
    assert about(command((0.0, 0.0, 0.0), goal_q=negated).angle, math.radians(20))


def test_large_rotation_does_not_slow_or_speed_up_translation():
    plain = command((1.0, 0.0, 0.0))
    turning = command((1.0, 0.0, 0.0), goal_q=rot_z(3.0))
    assert turning.linear == plain.linear


def test_large_translation_does_not_change_rotation_speed():
    plain = command((0.0, 0.0, 0.0), goal_q=rot_z(1.0))
    moving = command((1.0, 0.0, 0.0), goal_q=rot_z(1.0))
    assert moving.angular == plain.angular


def test_angular_direction_is_in_the_base_frame():
    # The arm is already turned 90 degrees about z; the goal is turned a further 30.
    cmd = command((0.0, 0.0, 0.0), current_q=rot_z(math.pi / 2),
                  goal_q=rot_z(math.pi / 2 + 0.3))
    assert about(cmd.angular[2], 0.3, tol=1e-9)


def test_setpoint_orientation_moves_toward_the_goal_and_stays_unit_length():
    cmd = command((0.0, 0.0, 0.0), goal_q=rot_z(1.0))
    assert about(control.norm(cmd.setpoint_orientation), 1.0)
    step = control.rotation_vector(IDENTITY, cmd.setpoint_orientation)
    # 1 rad away is over the angular cap, so the step is exactly one cycle at the cap
    assert about(step[2], LIMITS.max_angular_speed / LIMITS.publish_rate, tol=1e-9)


def test_rotation_vector_round_trips():
    for rotvec in [(0.1, 0.0, 0.0), (0.0, -0.7, 0.2), (1.0, 1.0, 1.0), (0.0, 0.0, 3.0)]:
        back = control.rotation_vector(IDENTITY, control.quat_from_rotation_vector(rotvec))
        assert all(about(a, b, tol=1e-9) for a, b in zip(back, rotvec))


def test_hold_command_is_zero_at_the_current_pose():
    cmd = control.hold_command((0.1, 0.2, 0.3), IDENTITY)
    assert cmd.linear == control.ZERO and cmd.angular == control.ZERO
    assert cmd.setpoint_position == (0.1, 0.2, 0.3)
    assert cmd.setpoint_orientation == IDENTITY


def test_workspace_box_includes_its_edges():
    lower, upper = (-0.7, -0.7, 0.0), (0.7, 0.7, 0.8)
    assert control.in_workspace((0.0, 0.0, 0.4), lower, upper)
    assert control.in_workspace((0.7, -0.7, 0.8), lower, upper)
    assert not control.in_workspace((0.71, 0.0, 0.4), lower, upper)
    assert not control.in_workspace((0.0, 0.0, -0.01), lower, upper)


def test_reached_needs_both_position_and_orientation():
    assert control.is_reached(0.005, 0.02, LIMITS)
    assert not control.is_reached(0.0051, 0.0, LIMITS)
    assert not control.is_reached(0.0, 0.0201, LIMITS)


@pytest.mark.parametrize('q', [(0.0, 0.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0),
                               (0.0, math.inf, 0.0, 1.0)])
def test_unusable_quaternions_are_refused(q):
    assert control.normalize_quat(q) is None


def test_quaternions_are_normalized():
    q = control.normalize_quat((0.0, 0.0, 0.0, 2.0))
    assert q == pytest.approx(IDENTITY)


def test_a_custom_limit_is_respected():
    fast = Limits(max_speed=0.2, gain=10.0)
    assert about(command((1.0, 0.0, 0.0), limits=fast).linear[0], 0.2)
