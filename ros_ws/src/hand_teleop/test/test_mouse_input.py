"""Check drag velocity, axis selection, and stopping without moving a robot."""

import math

import pytest

from hand_teleop.mouse_teleop_node import MouseInput


def test_inactive_input_cannot_move():
    controls = MouseInput(0.03)
    controls.move(100, 100)
    assert controls.velocity(0.05) == (0.0, 0.0, 0.0)


def test_click_anywhere_does_not_move():
    controls = MouseInput(0.03)
    controls.start(400, 100, 0.0)
    assert controls.velocity(0.05) == (0.0, 0.0, 0.0)


def test_faster_drag_produces_proportionally_faster_motion():
    controls = MouseInput(0.03)
    controls.start(250, 200, 0.0)
    controls.move(250, 195)
    slow = controls.velocity(0.1)[2]
    controls.move(250, 185)
    fast = controls.velocity(0.2)[2]
    assert slow == pytest.approx(0.005)
    assert fast == pytest.approx(2 * slow)


def test_same_drag_speed_is_independent_of_event_frequency():
    controls = MouseInput(0.03)
    controls.start(0, 0, 0.0)
    for x in range(1, 11):
        controls.move(x, 0)
    first = controls.velocity(0.1)
    controls.move(20, 0)
    assert controls.velocity(0.2) == pytest.approx(first)


def test_elapsed_time_changes_speed():
    controls = MouseInput(0.03)
    controls.start(0, 0, 0.0)
    controls.move(10, 0)
    first = controls.velocity(0.1)[1]
    controls.move(20, 0)
    assert controls.velocity(0.15)[1] == pytest.approx(2 * first)


def test_stationary_pointer_stops_even_while_button_is_held():
    controls = MouseInput(0.03)
    controls.start(250, 200, 0.0)
    controls.move(250, 190)
    assert controls.velocity(0.05)[2] > 0
    assert controls.active
    assert controls.velocity(0.1) == (0.0, 0.0, 0.0)


def test_shift_selects_x_and_discards_pending_z_motion():
    controls = MouseInput(0.03)
    controls.start(250, 200, 0.0)
    controls.move(250, 195)
    assert controls.velocity(0.05) == pytest.approx((0, 0, 0.01))
    controls.move(250, 190)
    controls.select_shift(True)
    assert controls.velocity(0.1) == (0.0, 0.0, 0.0)
    controls.move(250, 185)
    assert controls.velocity(0.15) == pytest.approx((0.01, 0, 0))
    controls.select_shift(False)
    controls.move(250, 180)
    assert controls.velocity(0.2) == pytest.approx((0, 0, 0.01))


@pytest.mark.parametrize('shift', [False, True])
def test_horizontal_motion_and_diagonal_speed_cap(shift):
    controls = MouseInput(0.03)
    controls.start(0, 0, 0.0)
    controls.select_shift(shift)
    controls.move(100, -100)
    velocity = controls.velocity(0.05)
    assert velocity[1] < 0
    assert velocity[0 if shift else 2] > 0
    assert velocity[2 if shift else 0] == 0
    assert math.sqrt(sum(v*v for v in velocity)) == pytest.approx(0.03)


def test_stop_requires_a_new_click():
    controls = MouseInput(0.03)
    controls.start(0, 0, 0.0)
    controls.move(10, -10)
    controls.stop()
    controls.move(20, -20)
    assert controls.velocity(0.05) == (0.0, 0.0, 0.0)
    controls.start(20, -20, 0.1)
    assert controls.velocity(0.15) == (0.0, 0.0, 0.0)


def test_stalled_event_loop_disarms_instead_of_replaying_drag():
    controls = MouseInput(0.03)
    controls.start(0, 0, 0.0)
    controls.move(100, 100)
    assert controls.velocity(0.3) == (0.0, 0.0, 0.0)
    assert not controls.active


@pytest.mark.parametrize('speed', [0.0, -0.01, float('inf'), float('nan')])
def test_invalid_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        MouseInput(speed)


@pytest.mark.parametrize('gain', [0.0, -0.01, float('inf'), float('nan')])
def test_invalid_gain_is_rejected(gain):
    with pytest.raises(ValueError):
        MouseInput(0.03, mouse_gain=gain)
