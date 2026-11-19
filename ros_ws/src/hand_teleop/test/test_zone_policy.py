"""Offline kinematic and policy tests; no ROS or controller connection."""

import unittest
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from hand_teleop.singularity_filter import SerialModel, SingularityFilter
from hand_teleop.recovery_hold import RecoveryHold


class PredictionTests(unittest.TestCase):
    def setUp(self):
        description = (Path(__file__).parent / 'fixtures/lite6_restricted.urdf').read_text()
        self.model = SerialModel(description)
        self.guard = SingularityFilter(self.model)
        self.angles = np.array([0., 0.5, 0.6, 0., 0.8, 0.])
        self.positions = dict(zip(self.model.names, self.angles))

    def test_prediction_does_not_change_measured_pose(self):
        original = self.angles.copy()
        prediction = self.guard.predict((0.001, 0., 0.), self.angles)
        np.testing.assert_array_equal(self.angles, original)
        self.assertEqual(len(prediction.conditions), 6)
        self.assertGreater(prediction.joint_margin, 0.1)
        self.assertLess(prediction.speed_usage, 1.)

    def test_stationary_prediction_matches_current_condition(self):
        prediction = self.guard.predict((0., 0., 0.), self.angles)
        expected = self.guard.condition(self.model.jacobian(self.angles))
        np.testing.assert_allclose(prediction.conditions, expected)

    def test_singular_and_invalid_inputs_fail_closed(self):
        for velocity, positions in [
            ((0.01, 0., 0.), dict.fromkeys(self.model.names, 0.)),
            ((float('nan'), 0., 0.), self.positions),
            ((0.01, 0., 0.), {}),
        ]:
            decision = self.guard.filter(velocity, positions)
            self.assertTrue(decision.blocked)
            self.assertEqual(decision.velocity, (0., 0., 0.))

    def test_outside_joint_bounds_fails_closed(self):
        self.positions['joint1'] = 4.
        decision = self.guard.filter((0., 0.01, 0.), self.positions)
        self.assertTrue(decision.blocked)
        self.assertIn('joint1', decision.reason)


class CautionTests(unittest.TestCase):
    def setUp(self):
        description = (Path(__file__).parent / 'fixtures/lite6_restricted.urdf').read_text()
        robot = ET.fromstring(description)
        limit = robot.find("joint[@name='joint1']/limit")
        limit.set('lower', str(-2 * math.pi))
        limit.set('upper', str(2 * math.pi))
        self.guard = SingularityFilter(SerialModel(ET.tostring(robot, encoding='unicode')))
        degrees = [204.4297475, -11.143872, 21.80863, 21.060791, 32.954406, -155.56641]
        self.positions = dict(zip(self.guard.model.names, map(math.radians, degrees)))

    def test_approach_slows_and_preserves_diagonal_direction(self):
        requested = np.array([0.01, 0.01, 0.])
        decision = self.guard.filter(requested, self.positions)
        self.assertFalse(decision.blocked)
        self.assertGreater(decision.speed_scale, 0.)
        self.assertLess(decision.speed_scale, 1.)
        np.testing.assert_allclose(decision.velocity, requested * decision.speed_scale)
        self.assertLess(decision.predicted_condition, self.guard.halt)

    def test_retreat_is_not_slowed(self):
        decision = self.guard.filter((-0.01, 0., 0.), self.positions)
        self.assertFalse(decision.blocked)
        self.assertEqual(decision.speed_scale, 1.)
        self.assertLess(decision.predicted_condition, decision.condition)

    def test_scale_is_continuous_and_monotonic(self):
        scales = [self.guard.approach_scale(value) for value in np.linspace(17., 25., 101)]
        self.assertEqual(scales[0], 1.)
        self.assertEqual(scales[-1], 0.)
        self.assertTrue(all(first >= second for first, second in zip(scales, scales[1:])))
        self.assertLess(max(abs(np.diff(scales))), 0.02)

    def test_zone_hysteresis(self):
        zones = [self.guard.classify(value) for value in [17.1, 16.9, 16.4, 25., 24.9, 24.4]]
        self.assertEqual(zones, ['CAUTION', 'CAUTION', 'NORMAL',
                                 'RECOVERY', 'RECOVERY', 'CAUTION'])

    def test_zero_is_never_replaced_by_motion(self):
        self.guard.filter((0.01, 0., 0.), self.positions)
        self.assertEqual(self.guard.filter((0., 0., 0.), self.positions).velocity,
                         (0., 0., 0.))


class RecoveryTests(PredictionTests):
    def setUp(self):
        super().setUp()
        self.angles = np.array([0.9788757726, 1.1884823155, 0.4157793029,
                                0.1621084306, -1.1742727766, 0.5167202791])
        self.positions = dict(zip(self.model.names, self.angles))

    def test_recovery_requires_explicit_enable(self):
        decision = self.guard.filter((0.01, 0., 0.), self.positions)
        self.assertTrue(decision.blocked)
        self.assertEqual(decision.zone, 'RECOVERY')

    def test_recovery_caps_speed_and_improves_each_step(self):
        decision = self.guard.filter((0.01, 0., 0.), self.positions, recovery_enabled=True)
        self.assertFalse(decision.blocked)
        self.assertLessEqual(np.linalg.norm(decision.velocity), 0.005)
        prediction = self.guard.predict(decision.velocity, self.angles)
        path = [decision.condition, *prediction.conditions]
        self.assertTrue(all(after < before - 1e-5 for before, after in zip(path, path[1:])))

    def test_recovery_rejects_worsening_direction(self):
        decision = self.guard.filter((-0.01, 0., 0.), self.positions, recovery_enabled=True)
        self.assertTrue(decision.blocked)
        self.assertEqual(decision.velocity, (0., 0., 0.))

    def test_hard_ceiling_cannot_be_bypassed(self):
        guard = SingularityFilter(self.model, recovery_ceiling=26.)
        self.assertTrue(guard.filter((0.01, 0., 0.), self.positions,
                                     recovery_enabled=True).blocked)

    def test_held_input_cannot_rearm_after_timeout_or_reset(self):
        hold = RecoveryHold()
        hold.update(True, 0.)
        self.assertFalse(hold.active(0.))
        hold.update(False, 0.)
        hold.update(True, 0.05)
        self.assertTrue(hold.active(0.1))
        hold.update(True, 0.5)
        self.assertFalse(hold.active(0.5))
        hold.update(False, 0.6)
        hold.update(True, 0.65)
        self.assertTrue(hold.active(0.7))
        hold.reset()
        hold.update(True, 0.75)
        self.assertFalse(hold.active(0.75))

    def test_invalid_recovery_settings_are_rejected(self):
        for settings in [{'recovery_speed': 0.1}, {'recovery_ceiling': 30.},
                         {'recovery_speed': float('nan')}, {'halt': 26.}]:
            with self.assertRaises(ValueError):
                SingularityFilter(self.model, **settings)


if __name__ == '__main__':
    unittest.main()
