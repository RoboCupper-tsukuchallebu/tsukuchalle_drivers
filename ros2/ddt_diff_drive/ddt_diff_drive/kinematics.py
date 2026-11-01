"""Differential drive math, free of ROS and hardware so it can be tested directly."""

import math
from dataclasses import dataclass

SECONDS_PER_MINUTE = 60.0


@dataclass(frozen=True)
class DriveConfig:
    wheel_radius_m: float
    track_width_m: float
    left_motor_id: int
    left_sign: int
    right_sign: int
    max_wheel_rpm: float
    max_acceleration_rpm_per_s: float


@dataclass(frozen=True)
class WheelTargets:
    left_rpm: float = 0.0
    right_rpm: float = 0.0


def validation_error(config: DriveConfig, firmware_max_rpm: int) -> str | None:
    """Return why the config is unusable, or None when it is valid."""
    if not (config.wheel_radius_m > 0 and math.isfinite(config.wheel_radius_m)):
        return "wheel_radius_m must be positive"
    if not (config.track_width_m > 0 and math.isfinite(config.track_width_m)):
        return "track_width_m must be positive"
    if config.left_motor_id not in (1, 2):
        return "left_motor_id must be 1 or 2"
    if config.left_sign not in (-1, 1) or config.right_sign not in (-1, 1):
        return "left_sign and right_sign must be 1 or -1"
    if not 0 < config.max_wheel_rpm <= firmware_max_rpm:
        return f"max_wheel_rpm must be in (0, {firmware_max_rpm}]"
    if not (config.max_acceleration_rpm_per_s > 0
            and math.isfinite(config.max_acceleration_rpm_per_s)):
        return "max_acceleration_rpm_per_s must be positive"
    return None


def twist_to_wheels(linear_m_s: float, angular_rad_s: float, config: DriveConfig) -> WheelTargets:
    """Convert a body twist to wheel speeds within the configured limit."""
    if not (math.isfinite(linear_m_s) and math.isfinite(angular_rad_s)):
        return WheelTargets()
    half_track_m = config.track_width_m / 2.0
    rpm_per_m_s = SECONDS_PER_MINUTE / (2.0 * math.pi * config.wheel_radius_m)
    left_rpm = (linear_m_s - angular_rad_s * half_track_m) * rpm_per_m_s
    right_rpm = (linear_m_s + angular_rad_s * half_track_m) * rpm_per_m_s
    # A huge but finite input can still overflow the arithmetic above.
    if not (math.isfinite(left_rpm) and math.isfinite(right_rpm)):
        return WheelTargets()
    return clamp_wheels(WheelTargets(left_rpm, right_rpm), config.max_wheel_rpm)


def clamp_wheels(wheels: WheelTargets, max_rpm: float) -> WheelTargets:
    """Scale both wheels together so that the commanded curve is preserved."""
    fastest_rpm = max(abs(wheels.left_rpm), abs(wheels.right_rpm))
    if fastest_rpm <= max_rpm:
        return wheels
    scale = max_rpm / fastest_rpm
    return WheelTargets(wheels.left_rpm * scale, wheels.right_rpm * scale)


def limit_rate(current: WheelTargets, target: WheelTargets, max_change_rpm: float) -> WheelTargets:
    """Move toward target; both wheels share one scale so the curve stays consistent."""
    left_change = target.left_rpm - current.left_rpm
    right_change = target.right_rpm - current.right_rpm
    largest_change = max(abs(left_change), abs(right_change))
    if largest_change <= max_change_rpm:
        return target
    scale = max_change_rpm / largest_change
    return WheelTargets(current.left_rpm + left_change * scale,
                        current.right_rpm + right_change * scale)


def to_motor_rpm(wheels: WheelTargets, config: DriveConfig) -> tuple[int, int]:
    """Apply wheel assignment and direction; returns (motor 1 rpm, motor 2 rpm)."""
    left_rpm = round(wheels.left_rpm * config.left_sign)
    right_rpm = round(wheels.right_rpm * config.right_sign)
    return (left_rpm, right_rpm) if config.left_motor_id == 1 else (right_rpm, left_rpm)
