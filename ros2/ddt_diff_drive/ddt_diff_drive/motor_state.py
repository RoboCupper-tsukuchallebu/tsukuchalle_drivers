"""Raw per-motor values of the latest feedback, in SI units. No ROS, no hardware."""

import math
from collections.abc import Mapping
from dataclasses import dataclass

from ddt_diff_drive.ddt_bridge import Feedback

RADIANS_PER_SECOND_PER_RPM = 2.0 * math.pi / 60.0


@dataclass(frozen=True)
class MotorStateSnapshot:
    """One entry per motor, ordered by motor id. Signs are the motor's own, not the wheel's."""

    names: tuple[str, ...]
    positions_rad: tuple[float, ...]  # Single-turn encoder angle, 0 to 2 pi.
    velocities_rad_per_s: tuple[float, ...]
    currents_amps: tuple[float, ...]
    newest_received_at: float  # time.monotonic() of the newest feedback in the snapshot.


def motor_name(motor_id: int) -> str:
    return f"motor_{motor_id}"


def build_motor_state_snapshot(feedbacks: Mapping[int, tuple[Feedback, float]], now: float,
                               stale_after_s: float) -> MotorStateSnapshot | None:
    """Collect the motors whose latest feedback is at most stale_after_s old. None if there is none."""
    fresh = [(feedback, received_at) for _, (feedback, received_at) in sorted(feedbacks.items())
             if now - received_at <= stale_after_s]
    if not fresh:
        return None
    return MotorStateSnapshot(
        names=tuple(motor_name(feedback.motor_id) for feedback, _ in fresh),
        positions_rad=tuple(math.radians(feedback.angle_degrees) for feedback, _ in fresh),
        velocities_rad_per_s=tuple(feedback.velocity_rpm * RADIANS_PER_SECOND_PER_RPM
                                   for feedback, _ in fresh),
        currents_amps=tuple(feedback.current_amps for feedback, _ in fresh),
        newest_received_at=max(received_at for _, received_at in fresh),
    )
