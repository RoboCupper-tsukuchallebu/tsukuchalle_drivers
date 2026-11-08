"""Wheel odometry math, free of ROS and hardware so it can be tested directly."""

import math
from dataclasses import dataclass, replace

from ddt_diff_drive.kinematics import SECONDS_PER_MINUTE, DriveConfig

# Odometry stops this long after the last feedback; also bounds how long a wheel's last
# value is held while the other wheel is silent (nav-stack contract: 0.1 s).
FEEDBACK_STALE_S = 0.1
VELOCITY_SIGMA_SLOPE = 0.02
YAW_RATE_SIGMA_SLOPE = 0.10  # Track width uncertainty of 10 %.
UNUSED_COVARIANCE = 1e6


@dataclass(frozen=True)
class OdometryPose:
    """Planar pose in the odom frame plus the wheel-derived twist of the last step."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    variance_x: float = 0.0
    variance_y: float = 0.0
    variance_yaw: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


def meters_per_second_per_rpm(config: DriveConfig) -> float:
    """Wheel speed of 1 rpm, which is also the quantization step of the feedback."""
    return 2.0 * math.pi * config.wheel_radius_m / SECONDS_PER_MINUTE


def wheel_rpm_from_motors(motor_rpms: dict[int, int], config: DriveConfig) -> tuple[float, float]:
    """Inverse of to_motor_rpm: motor id -> rpm becomes (left, right) wheel rpm, forward positive."""
    other_motor_id = 3 - config.left_motor_id
    return (motor_rpms[config.left_motor_id] * config.left_sign,
            motor_rpms[other_motor_id] * config.right_sign)


def twist_sigma(linear_m_s: float, angular_rad_s: float, config: DriveConfig) -> tuple[float, float]:
    """Standard deviations of (v, w): one-rpm quantization of each wheel plus a speed-relative part."""
    quantization_m_s = meters_per_second_per_rpm(config)
    linear_floor = quantization_m_s / math.sqrt(2.0)
    angular_floor = math.sqrt(2.0) * quantization_m_s / config.track_width_m
    return (linear_floor + VELOCITY_SIGMA_SLOPE * abs(linear_m_s),
            angular_floor + YAW_RATE_SIGMA_SLOPE * abs(angular_rad_s))


def integrate(pose: OdometryPose, left_rpm: float, right_rpm: float, dt_s: float,
              config: DriveConfig) -> OdometryPose:
    """Advance the pose by dt_s with the midpoint method; returns a new pose."""
    scale = meters_per_second_per_rpm(config)
    linear_m_s = (left_rpm + right_rpm) / 2.0 * scale
    angular_rad_s = (right_rpm - left_rpm) * scale / config.track_width_m
    distance_m = linear_m_s * dt_s
    yaw_change = angular_rad_s * dt_s
    midpoint_yaw = pose.yaw + yaw_change / 2.0
    yaw = math.remainder(pose.yaw + yaw_change, 2.0 * math.pi)
    moved = replace(
        pose,
        x=pose.x + distance_m * math.cos(midpoint_yaw),
        y=pose.y + distance_m * math.sin(midpoint_yaw),
        yaw=yaw,
        linear_velocity=linear_m_s,
        angular_velocity=angular_rad_s)
    if left_rpm == 0 and right_rpm == 0:
        return moved  # Standing still must not inflate the uncertainty.
    linear_sigma, angular_sigma = twist_sigma(linear_m_s, angular_rad_s, config)
    along_variance = (linear_sigma * dt_s) ** 2
    return replace(
        moved,
        variance_x=pose.variance_x + along_variance,
        variance_y=pose.variance_y + along_variance + distance_m ** 2 * pose.variance_yaw,
        variance_yaw=pose.variance_yaw + (angular_sigma * dt_s) ** 2)


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """(z, w) of a rotation about the vertical axis."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
