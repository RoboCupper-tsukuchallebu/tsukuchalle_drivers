"""Odometry math without ROS or hardware."""

import math
import unittest
from dataclasses import replace

from ddt_diff_drive.kinematics import DriveConfig
from ddt_diff_drive.odometry import (
    UNUSED_COVARIANCE,
    OdometryPose,
    integrate,
    twist_sigma,
    wheel_rpm_from_motors,
    yaw_to_quaternion,
)

WHEEL_RADIUS_M = 0.0935
TRACK_WIDTH_M = 0.26
METERS_PER_REVOLUTION = 2.0 * math.pi * WHEEL_RADIUS_M
CONFIG = DriveConfig(WHEEL_RADIUS_M, TRACK_WIDTH_M, left_motor_id=1, left_sign=1, right_sign=-1,
                     max_wheel_rpm=100.0, max_acceleration_rpm_per_s=100.0)


class WheelRpmFromMotorsTest(unittest.TestCase):
    def test_default_assignment_makes_both_wheels_forward(self):
        self.assertEqual(wheel_rpm_from_motors({1: 60, 2: -60}, CONFIG), (60, 60))

    def test_swapped_motor_ids_swap_the_wheels(self):
        config = replace(CONFIG, left_motor_id=2)
        self.assertEqual(wheel_rpm_from_motors({1: -60, 2: 30}, config), (30, 60))

    def test_signs_are_applied_per_wheel(self):
        config = replace(CONFIG, left_sign=-1, right_sign=1)
        self.assertEqual(wheel_rpm_from_motors({1: -60, 2: 60}, config), (60, 60))


class IntegrateTest(unittest.TestCase):
    def test_straight_line(self):
        pose = integrate(OdometryPose(), 60, 60, 1.0, CONFIG)
        self.assertAlmostEqual(pose.x, METERS_PER_REVOLUTION, delta=1e-6)
        self.assertAlmostEqual(METERS_PER_REVOLUTION, 0.5875, delta=1e-4)
        self.assertAlmostEqual(pose.y, 0.0, delta=1e-9)
        self.assertAlmostEqual(pose.yaw, 0.0, delta=1e-9)
        self.assertAlmostEqual(pose.linear_velocity, METERS_PER_REVOLUTION, delta=1e-6)

    def test_spin_in_place(self):
        pose = integrate(OdometryPose(), -60, 60, 0.1, CONFIG)
        expected_yaw = METERS_PER_REVOLUTION * 2.0 / TRACK_WIDTH_M * 0.1
        self.assertAlmostEqual(expected_yaw, 0.4519, delta=1e-4)
        self.assertAlmostEqual(pose.x, 0.0, delta=1e-9)
        self.assertAlmostEqual(pose.y, 0.0, delta=1e-9)
        self.assertAlmostEqual(pose.yaw, expected_yaw, delta=1e-6)
        self.assertAlmostEqual(pose.angular_velocity, expected_yaw / 0.1, delta=1e-6)

    def test_arc_matches_the_analytic_solution(self):
        pose = OdometryPose()
        for _ in range(100):
            pose = integrate(pose, 30, 60, 0.01, CONFIG)
        radius_m = TRACK_WIDTH_M / 2.0 * (60 + 30) / (60 - 30)
        angle_rad = METERS_PER_REVOLUTION * (60 - 30) / 60 / TRACK_WIDTH_M
        self.assertAlmostEqual(radius_m, 0.39, delta=1e-9)
        self.assertAlmostEqual(angle_rad, 1.1298, delta=1e-4)
        self.assertAlmostEqual(pose.x, radius_m * math.sin(angle_rad), delta=1e-3)
        self.assertAlmostEqual(pose.y, radius_m * (1 - math.cos(angle_rad)), delta=1e-3)
        self.assertAlmostEqual(pose.yaw, angle_rad, delta=1e-3)

    def test_yaw_wraps_to_minus_pi_side(self):
        pose = OdometryPose(yaw=math.pi - 0.01)
        pose = integrate(pose, -60, 60, 0.1, CONFIG)  # +0.4519 rad
        self.assertAlmostEqual(pose.yaw, -math.pi + 0.4519 - 0.01, delta=1e-3)
        self.assertTrue(-math.pi <= pose.yaw <= math.pi)

    def test_pose_is_not_mutated(self):
        start = OdometryPose()
        integrate(start, 60, 60, 1.0, CONFIG)
        self.assertEqual(start, OdometryPose())


class CovarianceTest(unittest.TestCase):
    def test_twist_sigma_at_rest_is_the_quantization_floor(self):
        linear_sigma, angular_sigma = twist_sigma(0.0, 0.0, CONFIG)
        self.assertAlmostEqual(linear_sigma, 0.006923, delta=1e-5)
        self.assertAlmostEqual(angular_sigma, 0.05326, delta=1e-5)

    def test_twist_sigma_grows_with_speed(self):
        linear_sigma, angular_sigma = twist_sigma(1.0, 2.0, CONFIG)
        self.assertAlmostEqual(linear_sigma, 0.026923, delta=1e-5)
        self.assertAlmostEqual(angular_sigma, 0.05326 + 0.2, delta=1e-5)

    def test_pose_variance_increases_monotonically_while_driving(self):
        pose = OdometryPose()
        for _ in range(20):
            next_pose = integrate(pose, 60, 30, 0.01, CONFIG)
            self.assertGreater(next_pose.variance_x, pose.variance_x)
            self.assertGreater(next_pose.variance_y, pose.variance_y)
            self.assertGreater(next_pose.variance_yaw, pose.variance_yaw)
            pose = next_pose

    def test_lateral_variance_includes_the_yaw_uncertainty(self):
        pose = integrate(OdometryPose(variance_yaw=0.01), 60, 60, 1.0, CONFIG)
        linear_sigma, _ = twist_sigma(METERS_PER_REVOLUTION, 0.0, CONFIG)
        expected_y = linear_sigma ** 2 + (METERS_PER_REVOLUTION ** 2) * 0.01
        self.assertAlmostEqual(pose.variance_y, expected_y, delta=1e-9)

    def test_standing_still_does_not_grow_the_pose_variance(self):
        driven = integrate(OdometryPose(), 60, 60, 0.1, CONFIG)
        still = integrate(driven, 0, 0, 0.1, CONFIG)
        self.assertEqual((still.variance_x, still.variance_y, still.variance_yaw),
                         (driven.variance_x, driven.variance_y, driven.variance_yaw))
        self.assertEqual((still.x, still.y, still.yaw), (driven.x, driven.y, driven.yaw))
        self.assertEqual((still.linear_velocity, still.angular_velocity), (0.0, 0.0))

    def test_unused_covariance_constant(self):
        self.assertEqual(UNUSED_COVARIANCE, 1e6)


class QuaternionTest(unittest.TestCase):
    def test_quarter_turn(self):
        z, w = yaw_to_quaternion(math.pi / 2.0)
        self.assertAlmostEqual(z, math.sin(math.pi / 4.0), delta=1e-12)
        self.assertAlmostEqual(w, math.cos(math.pi / 4.0), delta=1e-12)


if __name__ == "__main__":
    unittest.main()
