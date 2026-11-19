"""Exercise shared guard callbacks with ROS message types and no running robot."""

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from hand_teleop.servo_guard import ServoGuard
from hand_teleop.startup_pose import StartupPose

DESCRIPTION = (Path(__file__).parent / 'fixtures/lite6_restricted.urdf').read_text()
RECOVERY_POSE = [0.9788757726, 1.1884823155, 0.4157793029,
                 0.1621084306, -1.1742727766, 0.5167202791]


def host(allow_recovery=False):
    node = MagicMock()
    node.declare_parameter.side_effect = lambda name, default: SimpleNamespace(
        value=allow_recovery if name == 'allow_recovery' else default)
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    return node


def feedback(angles):
    message = JointState()
    message.name = [f'joint{index}' for index in range(1, 7)]
    message.position = list(angles)
    message.header.stamp.sec = 10
    return message


def ready_guard(allow_recovery=True):
    node = host(allow_recovery)
    stop = MagicMock()
    guard = ServoGuard(node, DESCRIPTION, on_stop=stop)
    with patch('hand_teleop.servo_guard.time.monotonic', return_value=1.):
        guard.joint_feedback(feedback(RECOVERY_POSE))
    return node, guard, stop


def press(guard):
    guard.hold.update(False, 1.01)
    guard.hold.update(True, 1.02)


def test_recovery_entry_discards_old_input_then_requires_release_and_hold():
    node, guard, stop = ready_guard()
    press(guard)
    decision = guard.decide((0.01, 0., 0.), 1.03)
    assert decision.blocked and decision.velocity == (0., 0., 0.)
    assert not guard.hold.active(1.03)
    stop.assert_called()
    press(guard)
    decision = guard.decide((0.01, 0., 0.), 1.04)
    assert not decision.blocked
    assert decision.velocity == (0.005, 0., 0.)
    status = json.loads(node.create_publisher.return_value.publish.call_args.args[0].data)
    assert status['zone'] == 'RECOVERY'
    assert status['speed_scale'] == 0.5


def test_recovery_is_disabled_by_default_even_with_hold():
    _, guard, _ = ready_guard(False)
    guard.decide((0.01, 0., 0.), 1.01)
    press(guard)
    assert guard.decide((0.01, 0., 0.), 1.04).blocked


def test_feedback_loss_revokes_hold_and_clears_input():
    _, guard, stop = ready_guard()
    guard.decide((0.01, 0., 0.), 1.01)
    press(guard)
    assert not guard.decide((0.01, 0., 0.), 1.04).blocked
    assert guard.decide((0.01, 0., 0.), 1.5).blocked
    stop.assert_called()
    assert not guard.hold.active(1.5)


def test_zero_request_does_not_trigger_escape_motion():
    _, guard, _ = ready_guard()
    guard.decide((0., 0., 0.), 1.01)
    press(guard)
    assert guard.decide((0., 0., 0.), 1.04).velocity == (0., 0., 0.)


@pytest.mark.parametrize('angles,blocked', [
    ([0., 0.5, 0.6, 0., 0.8, 0.], False),
    ([0.] * 6, True),
    (RECOVERY_POSE, True),
    ([4., 0.5, 0.6, 0., 0.8, 0.], True),
])
def test_startup_assesses_measured_pose_without_publishing(angles, blocked):
    node = host()
    monitor = StartupPose(node)
    monitor.model(String(data=DESCRIPTION))
    with patch('hand_teleop.startup_pose.time.monotonic', return_value=1.):
        monitor.feedback(feedback(angles))
        assert monitor.assess().blocked is blocked
    node.create_publisher.assert_not_called()
    node.create_client.assert_not_called()


def test_startup_requires_model_and_fresh_feedback():
    monitor = StartupPose(host())
    assert monitor.assess().blocked
    monitor.model(String(data=DESCRIPTION))
    assert monitor.assess().blocked
    with patch('hand_teleop.startup_pose.time.monotonic', return_value=1.):
        monitor.feedback(feedback([0., 0.5, 0.6, 0., 0.8, 0.]))
    with patch('hand_teleop.startup_pose.time.monotonic', return_value=2.):
        assert monitor.assess().blocked
