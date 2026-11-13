"""Exercise keyboard deadman and sensitivity changes without connecting an arm."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from hand_teleop.mouse_teleop_node import MouseInput, MouseTeleop


@pytest.fixture
def node():
    result = object.__new__(MouseTeleop)
    result.controls = MouseInput(0.1, 0.0002)
    result.base_mouse_gain = 0.0002
    result.space_held = False
    result.space_release_pending = None
    result.shift_keys = set()
    result.canvas = MagicMock()
    result.canvas.winfo_pointerx.return_value = 150
    result.canvas.winfo_pointery.return_value = 150
    result.canvas.winfo_rootx.return_value = 100
    result.canvas.winfo_rooty.return_value = 100
    result.root = MagicMock()
    result.status = MagicMock()
    result.publish = MagicMock()
    return result


def press(node, state=0):
    with patch('hand_teleop.mouse_teleop_node.time.monotonic', return_value=0):
        node.space_press(SimpleNamespace(state=state))


def test_space_arms_without_click_and_only_pointer_motion_requests_motion(node):
    press(node)
    assert node.controls.active
    assert node.controls.velocity(0.05) == (0, 0, 0)
    node.controls.move(50, 40)
    assert node.controls.velocity(0.1)[2] == pytest.approx(0.04)


def test_shift_space_selects_x(node):
    press(node, 1)
    node.controls.move(50, 40)
    assert node.controls.velocity(0.05) == pytest.approx((0.04, 0, 0))


def test_space_release_stops_and_requires_another_press(node):
    press(node)
    node.space_release(SimpleNamespace())
    callback = node.root.after_idle.call_args.args[0]
    callback()
    assert not node.controls.active and not node.space_held
    node.publish.assert_called_with((0, 0, 0))


def test_auto_repeat_does_not_reset_pointer_or_consume_drag(node):
    press(node)
    node.controls.move(50, 40)
    node.space_release(SimpleNamespace())
    token = node.space_release_pending
    press(node)
    node.root.after_cancel.assert_called_with(token)
    assert node.controls.velocity(0.05) == pytest.approx((0, 0, 0.04))


def test_leaving_pad_cannot_rearm_on_keyboard_repeat(node):
    press(node)
    node.stop()  # Pointer leave/focus loss uses this same stop.
    press(node)
    assert not node.controls.active
    node.finish_space_release()
    press(node)
    assert node.controls.active


def test_space_outside_pad_cannot_arm_on_repeated_keypress(node):
    node.canvas.winfo_pointerx.return_value = 50
    press(node)
    assert not node.controls.active
    node.canvas.winfo_pointerx.return_value = 150
    press(node)
    assert not node.controls.active


def test_sensitivity_change_disarms_and_preserves_total_speed_cap(node):
    press(node)
    node.controls.move(50, 40)
    node.change_sensitivity('5')
    assert not node.controls.active
    assert node.controls.mouse_gain == pytest.approx(0.001)
    node.controls.start(50, 50, 0)
    node.controls.move(50, 40)
    assert node.controls.velocity(0.05) == pytest.approx((0, 0, 0.1))
