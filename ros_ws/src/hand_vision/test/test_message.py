from types import SimpleNamespace

from builtin_interfaces.msg import Time
from hand_vision.message import build_message
import pytest


def test_untracked_message_is_all_zero():
    msg = build_message(Time(sec=1))
    assert not msg.tracked
    assert msg.handedness == ''
    assert msg.header.frame_id == 'camera'
    assert len(msg.landmarks) == 21
    assert all(p.x == p.y == p.z == 0.0 for p in msg.landmarks)


def test_tracked_message_copies_landmarks():
    points = [SimpleNamespace(x=i / 100, y=0.5, z=-0.01) for i in range(21)]
    msg = build_message(Time(sec=2, nanosec=5), points, 'Right')
    assert msg.tracked
    assert msg.handedness == 'Right'
    assert msg.header.stamp.sec == 2
    assert msg.landmarks[0].x == 0.0
    assert msg.landmarks[20].x == pytest.approx(0.2)
    assert msg.landmarks[20].z == pytest.approx(-0.01)


def test_wrong_landmark_count_is_rejected():
    with pytest.raises(ValueError):
        build_message(Time(), [SimpleNamespace(x=0, y=0, z=0)] * 5, 'Left')
