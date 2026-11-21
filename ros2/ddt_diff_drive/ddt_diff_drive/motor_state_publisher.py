"""Publishes /wheel/motor_states: the raw feedback of each motor (speed, current, encoder angle).

Read-only: it reads the latest feedback kept by BridgeTelemetry and never touches the serial port
or the motor commands. Publishing is driven by the control timer, like the wheel odometry.
"""

import time

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState

from ddt_diff_drive.motor_state import MotorStateSnapshot, build_motor_state_snapshot
from ddt_diff_drive.odometry import FEEDBACK_STALE_S
from ddt_diff_drive.telemetry import BridgeTelemetry

MOTOR_STATE_TOPIC = "/wheel/motor_states"
MOTOR_STATE_QOS_DEPTH = 10
NANOSECONDS_PER_SECOND = 1_000_000_000


class MotorStatePublisher:
    def __init__(self, node, telemetry: BridgeTelemetry):
        self._node = node
        self._telemetry = telemetry
        self._publisher = node.create_publisher(JointState, MOTOR_STATE_TOPIC, MOTOR_STATE_QOS_DEPTH)
        self._last_published_received_at = None  # time.monotonic() of the newest feedback already sent.

    def publish(self) -> None:
        """Publish the latest feedback if a new one arrived. Never raises: it runs in the control timer."""
        try:
            self._publish_if_new(time.monotonic())
        except Exception as error:  # noqa: BLE001 - telemetry must not stop the drive control
            self._node.get_logger().error(f"motor state publish failed: {error!r}",
                                          throttle_duration_sec=1.0)

    def _publish_if_new(self, now: float) -> None:
        snapshot = build_motor_state_snapshot(self._telemetry.feedbacks, now, FEEDBACK_STALE_S)
        if snapshot is None:
            return
        if (self._last_published_received_at is not None
                and snapshot.newest_received_at <= self._last_published_received_at):
            return
        self._last_published_received_at = snapshot.newest_received_at
        self._publisher.publish(self._to_message(snapshot, now))

    def _to_message(self, snapshot: MotorStateSnapshot, now: float) -> JointState:
        # The stamp is the ROS time at which the newest feedback was received, not the publish time.
        age_ns = int((now - snapshot.newest_received_at) * NANOSECONDS_PER_SECOND)
        stamp_ns = self._node.get_clock().now().nanoseconds - age_ns
        message = JointState()
        message.header.stamp = Time(sec=stamp_ns // NANOSECONDS_PER_SECOND,
                                    nanosec=stamp_ns % NANOSECONDS_PER_SECOND)
        message.name = list(snapshot.names)
        message.position = list(snapshot.positions_rad)
        message.velocity = list(snapshot.velocities_rad_per_s)
        # JointState has no field for current: effort carries the motor current in amperes, not a torque.
        message.effort = list(snapshot.currents_amps)
        return message
