"""Offline kinematic and policy tests; no ROS or controller connection."""

import unittest
from pathlib import Path

import numpy as np

from hand_teleop.singularity_filter import SerialModel, SingularityFilter


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


if __name__ == '__main__':
    unittest.main()
