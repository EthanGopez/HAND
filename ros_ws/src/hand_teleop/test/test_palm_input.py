"""Check palm-to-velocity mapping and its rejections without a robot."""

import math

import pytest

from hand_teleop.palm_input import PalmInput


def feed(controls, samples):
    """Push (x, y, z, t) samples and return the resulting velocity."""
    for sample in samples:
        controls.update(*sample)
    return controls.velocity()


def test_a_single_sample_cannot_move_the_arm():
    controls = PalmInput(0.05)
    controls.update(10.0, 20.0, 40.0, 0.0)
    assert controls.velocity() == (0.0, 0.0, 0.0)


def test_position_alone_commands_nothing():
    """A hand parked far off centre is not a movement command."""
    controls = PalmInput(0.05)
    for index in range(10):
        controls.update(15.0, -12.0, 40.0, index * 0.04)
    assert controls.velocity() == (0.0, 0.0, 0.0)


def test_hand_right_moves_tool_along_negative_y():
    controls = PalmInput(0.5)
    velocity = feed(controls, [(0.0, 0.0, 40.0, 0.0), (1.0, 0.0, 40.0, 0.1)])
    assert velocity[1] == pytest.approx(-0.1)
    assert velocity[0] == 0.0 and velocity[2] == 0.0


def test_hand_up_moves_tool_along_positive_z():
    controls = PalmInput(0.5)
    # Image y grows downward, so a rising hand has a decreasing y.
    velocity = feed(controls, [(0.0, 0.0, 40.0, 0.0), (0.0, -1.0, 40.0, 0.1)])
    assert velocity[2] == pytest.approx(0.1)


def test_depth_is_ignored_until_its_scale_is_set():
    controls = PalmInput(0.5)
    assert feed(controls, [(0.0, 0.0, 40.0, 0.0), (0.0, 0.0, 40.2, 0.1)])[0] == 0.0
    enabled = PalmInput(0.5, scale_x=0.01)
    # Depth grows with distance, so approaching the camera is +X.
    velocity = feed(enabled, [(0.0, 0.0, 40.0, 0.0), (0.0, 0.0, 39.0, 0.1)])
    assert velocity[0] == pytest.approx(0.1)


def test_faster_hand_produces_proportionally_faster_motion():
    slow = feed(PalmInput(0.5), [(0.0, 0.0, 40.0, 0.0), (0.5, 0.0, 40.0, 0.1)])
    fast = feed(PalmInput(0.5), [(0.0, 0.0, 40.0, 0.0), (1.0, 0.0, 40.0, 0.1)])
    assert fast[1] == pytest.approx(2 * slow[1])


def test_speed_is_independent_of_the_sample_rate():
    """The same hand motion over the same time is the same command."""
    sparse = feed(PalmInput(0.5), [(0.0, 0.0, 40.0, 0.0), (1.0, 0.0, 40.0, 0.1)])
    dense = feed(PalmInput(0.5), [(index * 0.2, 0.0, 40.0, index * 0.02)
                                  for index in range(6)])
    assert dense[1] == pytest.approx(sparse[1])


def test_displacement_is_consumed_once():
    controls = PalmInput(0.5)
    assert feed(controls, [(0.0, 0.0, 40.0, 0.0), (1.0, 0.0, 40.0, 0.1)])[1] != 0.0
    # The hand is still visible and still off centre, but it stopped moving.
    assert controls.velocity() == (0.0, 0.0, 0.0)


def test_total_speed_is_capped_on_diagonals():
    controls = PalmInput(0.05)
    velocity = feed(controls, [(0.0, 0.0, 40.0, 0.0), (5.0, 5.0, 40.0, 0.1)])
    assert math.hypot(*velocity) == pytest.approx(0.05)
    # The direction survives the cap.
    assert velocity[1] == pytest.approx(velocity[2])


def test_stop_requires_a_new_reference_before_moving_again():
    controls = PalmInput(0.5)
    controls.update(0.0, 0.0, 40.0, 0.0)
    controls.stop()
    assert not controls.active
    controls.update(1.0, 0.0, 40.0, 0.1)
    assert controls.velocity() == (0.0, 0.0, 0.0)
    assert controls.active


def test_tracking_glitch_does_not_become_a_lunge():
    controls = PalmInput(0.5, max_jump=6.0)
    controls.update(0.0, 0.0, 40.0, 0.0)
    # The recorded CSV contains single-frame jumps of over a metre, caused by
    # the palm-width ruler collapsing on a bad detection.
    assert controls.update(150.0, 0.0, 40.0, 0.04) is False
    assert controls.velocity() == (0.0, 0.0, 0.0)
    assert controls.rejected == 1
    # The glitch position became the new reference, so tracking continues.
    controls.update(151.0, 0.0, 40.0, 0.08)
    assert controls.velocity()[1] == pytest.approx(-0.25)


def test_hand_lost_and_refound_does_not_replay_the_gap():
    controls = PalmInput(0.5, max_gap=0.5)
    controls.update(0.0, 0.0, 40.0, 0.0)
    # The real recording contains gaps of twenty seconds while no hand is
    # visible; the hand reappears somewhere else entirely.
    assert controls.update(20.0, 10.0, 40.0, 24.0) is False
    assert controls.velocity() == (0.0, 0.0, 0.0)


def test_repeated_timestamp_is_rejected():
    controls = PalmInput(0.5)
    controls.update(0.0, 0.0, 40.0, 1.0)
    assert controls.update(5.0, 0.0, 40.0, 1.0) is False
    assert controls.velocity() == (0.0, 0.0, 0.0)


def test_source_clock_restart_re_references_instead_of_locking_up():
    controls = PalmInput(0.5)
    controls.update(0.0, 0.0, 40.0, 5000.0)
    assert controls.update(0.0, 0.0, 40.0, 2.0) is False
    # A later sample from the new session must move the arm again.
    controls.update(1.0, 0.0, 40.0, 2.1)
    assert controls.velocity()[1] == pytest.approx(-0.1)


@pytest.mark.parametrize('bad', [
    (float('nan'), 0.0, 40.0, 0.1),
    (0.0, float('inf'), 40.0, 0.1),
    (0.0, 0.0, 40.0, float('nan')),
])
def test_non_finite_sample_stops_instead_of_guessing(bad):
    controls = PalmInput(0.5)
    controls.update(0.0, 0.0, 40.0, 0.0)
    assert controls.update(*bad) is False
    assert controls.velocity() == (0.0, 0.0, 0.0)
    assert not controls.active


def test_deadband_ignores_jitter():
    controls = PalmInput(0.5, deadband=0.2)
    assert feed(controls, [(0.0, 0.0, 40.0, 0.0), (0.1, 0.0, 40.0, 0.1)])[1] == 0.0
    assert feed(controls, [(0.1, 0.0, 40.0, 0.2), (1.0, 0.0, 40.0, 0.3)])[1] != 0.0


def test_depth_noise_does_not_veto_sideways_motion():
    """A wild depth reading must not cancel the motion the camera did see."""
    controls = PalmInput(0.5, scale_x=0.01, max_depth_jump=1.0)
    controls.update(0.0, 0.0, 40.0, 0.0)
    assert controls.update(1.0, 0.0, 79.0, 0.1) is True
    velocity = controls.velocity()
    assert velocity[1] == pytest.approx(-0.1)   # sideways command survives
    assert velocity[0] == 0.0                   # the depth step is discarded
    assert controls.depth_rejected == 1
    assert controls.rejected == 0


def test_image_plane_jump_still_rejects_the_whole_sample():
    controls = PalmInput(0.5, scale_x=0.01, max_jump=6.0)
    controls.update(0.0, 0.0, 40.0, 0.0)
    assert controls.update(50.0, 0.0, 40.1, 0.1) is False
    assert controls.velocity() == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'max_speed': 0.0}, {'max_speed': -1.0}, {'max_speed': float('nan')},
    {'max_speed': 0.05, 'max_jump': 0.0}, {'max_speed': 0.05, 'max_gap': -1.0},
    {'max_speed': 0.05, 'max_depth_jump': 0.0},
    {'max_speed': 0.05, 'deadband': -0.1},
])
def test_invalid_settings_are_refused(kwargs):
    with pytest.raises(ValueError):
        PalmInput(**kwargs)
