from types import SimpleNamespace

from hand_vision.tripwire import TripwireGate
import pytest

W, H = 640, 480


def hand(x, y):
    """All 21 landmarks at one normalized point."""
    return [SimpleNamespace(x=x, y=y, z=0.0) for _ in range(21)]


def test_hand_outside_zone_does_not_trigger():
    gate = TripwireGate(0.15, 5.0)
    assert not gate.update([hand(0.1, 0.1)], W, H, 0.0)
    assert not gate.update([], W, H, 1.0)


def test_any_hand_in_zone_triggers():
    gate = TripwireGate(0.15, 5.0)
    assert gate.update([hand(0.1, 0.1), hand(0.52, 0.5)], W, H, 0.0)


def test_hand_anywhere_keeps_tracking_and_resets_timer():
    gate = TripwireGate(0.15, 5.0)
    gate.update([hand(0.5, 0.5)], W, H, 0.0)
    assert gate.update([hand(0.05, 0.9)], W, H, 1.0)
    assert gate.update([], W, H, 5.9)
    assert gate.seconds_lost(5.9) == pytest.approx(4.9)
    assert gate.update([hand(0.9, 0.9)], W, H, 6.0)
    assert gate.update([], W, H, 10.9)


def test_lost_timeout_resets_and_needs_zone_again():
    gate = TripwireGate(0.15, 5.0)
    gate.update([hand(0.5, 0.5)], W, H, 0.0)
    assert not gate.update([], W, H, 5.0)
    assert not gate.update([hand(0.9, 0.9)], W, H, 6.0)
    assert gate.update([hand(0.5, 0.5)], W, H, 7.0)


def test_rejects_bad_settings():
    with pytest.raises(ValueError):
        TripwireGate(0.0, 5.0)
    with pytest.raises(ValueError):
        TripwireGate(0.15, -1.0)
