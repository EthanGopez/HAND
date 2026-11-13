"""Hardware health state transitions with simulated clocks and joint messages."""
import pytest
from sensor_msgs.msg import JointState
from hand_bringup.hardware_watchdog import FeedbackHealth


def sample(stamp=10_000_000_000):
    msg = JointState()
    msg.name = [f'joint{i}' for i in range(1,7)]
    msg.position = [0.1]*6
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return msg


def test_successful_feedback_keeps_watchdog_alive():
    health = FeedbackHealth(0)
    health.observe(sample(), 0.1, 10_000_000_000)
    assert health.check(0.4, 10_300_000_000) is None
    health.observe(sample(10_400_000_000), 0.5, 10_400_000_000)
    assert health.check(0.8, 10_700_000_000) is None


def test_read_timeouts_latch_fault_and_restoration_cannot_resume():
    health = FeedbackHealth(0)
    health.observe(sample(), 0.1, 10_000_000_000)
    failure = health.check(0.7, 10_600_000_000)
    assert failure
    health.observe(sample(10_700_000_000), 0.8, 10_700_000_000)
    assert health.check(0.8, 10_700_000_000) == failure


def test_no_samples_after_startup_causes_failure():
    assert FeedbackHealth(0).check(0.6, 10_000_000_000)


def test_replayed_samples_do_not_refresh_watchdog():
    health = FeedbackHealth(0)
    health.observe(sample(), 0.1, 10_000_000_000)
    health.observe(sample(), 0.5, 10_400_000_000)
    assert health.check(0.7, 10_600_000_000)


@pytest.mark.parametrize('stamp', [0, 11_000_000_000, 9_000_000_000])
def test_invalid_sample_clock_is_a_fault(stamp):
    health = FeedbackHealth(0)
    health.observe(sample(stamp), 0.1, 10_000_000_000)
    assert health.check(0.1, 10_000_000_000)


def test_clock_jump_is_a_fault():
    health = FeedbackHealth(0)
    health.observe(sample(), 0.1, 10_000_000_000)
    assert health.check(0.2, 9_900_000_000)


def test_zero_positions_are_permitted():
    health = FeedbackHealth(0)
    msg = sample();msg.position = [0.0]*6
    health.observe(msg, 0.1, 10_000_000_000)
    assert health.check(0.2, 10_100_000_000) is None
