"""Raw motor state values without ROS or hardware."""

import math
import unittest

from ddt_diff_drive.ddt_bridge import Feedback
from ddt_diff_drive.motor_state import build_motor_state_snapshot

STALE_AFTER_S = 0.1
NOW = 100.0


def feedback(motor_id: int, velocity_rpm: int = 0, current_amps: float = 0.0,
             angle_degrees: float = 0.0) -> Feedback:
    return Feedback(motor_id, velocity_rpm, current_amps, angle_degrees, fault=0, mode=2)


class BuildMotorStateSnapshotTest(unittest.TestCase):
    def test_no_feedback_gives_no_snapshot(self):
        self.assertIsNone(build_motor_state_snapshot({}, NOW, STALE_AFTER_S))

    def test_values_are_converted_to_si_units_and_keep_the_raw_motor_sign(self):
        feedbacks = {
            1: (feedback(1, velocity_rpm=60, current_amps=1.5, angle_degrees=180.0), NOW - 0.01),
            2: (feedback(2, velocity_rpm=-30, current_amps=-0.25, angle_degrees=90.0), NOW - 0.02),
        }

        snapshot = build_motor_state_snapshot(feedbacks, NOW, STALE_AFTER_S)

        self.assertEqual(snapshot.names, ("motor_1", "motor_2"))
        self.assertAlmostEqual(snapshot.velocities_rad_per_s[0], 2.0 * math.pi)
        self.assertAlmostEqual(snapshot.velocities_rad_per_s[1], -math.pi)
        self.assertAlmostEqual(snapshot.positions_rad[0], math.pi)
        self.assertAlmostEqual(snapshot.positions_rad[1], math.pi / 2.0)
        self.assertEqual(snapshot.currents_amps, (1.5, -0.25))

    def test_motors_are_ordered_by_id_whatever_the_arrival_order(self):
        feedbacks = {2: (feedback(2), NOW), 1: (feedback(1), NOW)}

        snapshot = build_motor_state_snapshot(feedbacks, NOW, STALE_AFTER_S)

        self.assertEqual(snapshot.names, ("motor_1", "motor_2"))

    def test_newest_reception_time_is_reported(self):
        feedbacks = {1: (feedback(1), NOW - 0.03), 2: (feedback(2), NOW - 0.01)}

        snapshot = build_motor_state_snapshot(feedbacks, NOW, STALE_AFTER_S)

        self.assertEqual(snapshot.newest_received_at, NOW - 0.01)

    def test_a_stale_motor_is_left_out(self):
        feedbacks = {1: (feedback(1), NOW - 0.5), 2: (feedback(2, velocity_rpm=10), NOW - 0.01)}

        snapshot = build_motor_state_snapshot(feedbacks, NOW, STALE_AFTER_S)

        self.assertEqual(snapshot.names, ("motor_2",))
        self.assertEqual(len(snapshot.velocities_rad_per_s), 1)

    def test_only_stale_feedback_gives_no_snapshot(self):
        feedbacks = {1: (feedback(1), NOW - 0.5), 2: (feedback(2), NOW - 0.2)}

        self.assertIsNone(build_motor_state_snapshot(feedbacks, NOW, STALE_AFTER_S))


if __name__ == "__main__":
    unittest.main()
