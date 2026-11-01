import math
import unittest

from ddt_diff_drive.kinematics import (
    DriveConfig,
    WheelTargets,
    limit_rate,
    to_motor_rpm,
    twist_to_wheels,
    validation_error,
)

RADIUS_M = 0.1
CONFIG = DriveConfig(
    wheel_radius_m=RADIUS_M,
    track_width_m=0.4,
    left_motor_id=1,
    left_sign=1,
    right_sign=-1,
    max_wheel_rpm=100.0,
    max_acceleration_rpm_per_s=100.0,
)
RPM_AT_ONE_METER_PER_SECOND = 60.0 / (2.0 * math.pi * RADIUS_M)


def replace_config(**changes) -> DriveConfig:
    return DriveConfig(**{**CONFIG.__dict__, **changes})


class TwistToWheelsTest(unittest.TestCase):
    def test_straight_line_moves_wheels_equally(self):
        wheels = twist_to_wheels(0.5, 0.0, CONFIG)
        self.assertAlmostEqual(wheels.left_rpm, 0.5 * RPM_AT_ONE_METER_PER_SECOND)
        self.assertAlmostEqual(wheels.right_rpm, wheels.left_rpm)

    def test_positive_angular_turns_left(self):
        wheels = twist_to_wheels(0.0, 1.0, CONFIG)
        self.assertAlmostEqual(wheels.right_rpm, 0.2 * RPM_AT_ONE_METER_PER_SECOND)
        self.assertAlmostEqual(wheels.left_rpm, -wheels.right_rpm)

    def test_saturation_preserves_turn_ratio(self):
        unlimited = twist_to_wheels(5.0, 5.0, replace_config(max_wheel_rpm=1e9))
        limited = twist_to_wheels(5.0, 5.0, CONFIG)
        self.assertAlmostEqual(limited.right_rpm, 100.0)
        self.assertAlmostEqual(limited.left_rpm / limited.right_rpm,
                               unlimited.left_rpm / unlimited.right_rpm)

    def test_non_finite_command_stops(self):
        self.assertEqual(twist_to_wheels(math.nan, 0.0, CONFIG), WheelTargets())
        self.assertEqual(twist_to_wheels(0.0, math.inf, CONFIG), WheelTargets())

    def test_overflowing_command_stops(self):
        self.assertEqual(twist_to_wheels(1e308, 0.0, CONFIG), WheelTargets())
        self.assertEqual(twist_to_wheels(1e308, -1e308, CONFIG), WheelTargets())


class LimitRateTest(unittest.TestCase):
    def test_reaches_nearby_target(self):
        target = WheelTargets(1.0, -1.0)
        self.assertEqual(limit_rate(WheelTargets(), target, 2.0), target)

    def test_limits_largest_change_and_keeps_ratio(self):
        result = limit_rate(WheelTargets(), WheelTargets(100.0, 50.0), 10.0)
        self.assertAlmostEqual(result.left_rpm, 10.0)
        self.assertAlmostEqual(result.right_rpm, 5.0)

    def test_limits_deceleration_too(self):
        result = limit_rate(WheelTargets(100.0, 100.0), WheelTargets(), 10.0)
        self.assertAlmostEqual(result.left_rpm, 90.0)


class ToMotorRpmTest(unittest.TestCase):
    def test_applies_signs(self):
        self.assertEqual(to_motor_rpm(WheelTargets(10.0, 20.0), CONFIG), (10, -20))

    def test_swaps_motors(self):
        config = replace_config(left_motor_id=2)
        self.assertEqual(to_motor_rpm(WheelTargets(10.0, 20.0), config), (-20, 10))


class ValidationTest(unittest.TestCase):
    def test_accepts_default(self):
        self.assertIsNone(validation_error(CONFIG, 210))

    def test_rejects_bad_values(self):
        for changes in (
            {"wheel_radius_m": 0.0},
            {"track_width_m": -1.0},
            {"track_width_m": math.nan},
            {"left_motor_id": 3},
            {"left_sign": 0},
            {"right_sign": 2},
            {"max_wheel_rpm": 211.0},
            {"max_wheel_rpm": 0.0},
            {"max_acceleration_rpm_per_s": 0.0},
        ):
            with self.subTest(changes):
                self.assertIsNotNone(validation_error(replace_config(**changes), 210))


if __name__ == "__main__":
    unittest.main()
