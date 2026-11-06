"""Publishes /wheel/odom and TF odom->base_footprint from motor feedback.

Integration is driven by feedback (on_feedback); publishing is driven by the control timer
(publish). The two never touch the serial port.
"""

import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from ddt_diff_drive.ddt_bridge import Feedback
from ddt_diff_drive.odometry import (
    FEEDBACK_STALE_S,
    UNUSED_COVARIANCE,
    OdometryPose,
    integrate,
    twist_sigma,
    wheel_rpm_from_motors,
    yaw_to_quaternion,
)

ODOM_TOPIC = "/wheel/odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
ODOM_QOS_DEPTH = 10
MOTOR_IDS = (1, 2)
NANOSECONDS_PER_SECOND = 1_000_000_000
# Row-major 6x6 diagonal indexes: x/y/z/roll/pitch/yaw.
DIAGONAL = {name: index * 7 for index, name in enumerate(("x", "y", "z", "roll", "pitch", "yaw"))}


class OdometryPublisher:
    def __init__(self, node, config_getter):
        self._node = node
        self._config_getter = config_getter
        self._odom_publisher = node.create_publisher(Odometry, ODOM_TOPIC, ODOM_QOS_DEPTH)
        self._tf_broadcaster = TransformBroadcaster(node)
        self.is_tf_enabled = True
        self._pose = OdometryPose()
        self._latest_rpm: dict[int, tuple[int, float]] = {}  # motor id -> (rpm, received_at)
        self._reference_time = None  # monotonic time of the previous feedback frame
        self._last_integrated_at = None
        self._stamp_ns = 0  # ROS time of the last integration step
        self._has_unpublished_step = False
        self._last_published_ns = None
        self._paused_at = None
        self._is_clock_warned = False

    def on_feedback(self, feedback: Feedback, now: float) -> None:
        """Integrate one frame. Never raises: it runs inside the serial poll."""
        try:
            self._integrate_frame(feedback, now)
        except Exception as error:  # noqa: BLE001 - must not break poll() and the control loop
            self._node.get_logger().error(f"wheel odometry integration failed: {error!r}",
                                          throttle_duration_sec=1.0)

    def _integrate_frame(self, feedback: Feedback, now: float) -> None:
        if feedback.motor_id not in MOTOR_IDS:
            return
        self._latest_rpm[feedback.motor_id] = (feedback.velocity_rpm, now)
        reference_time, self._reference_time = self._reference_time, now
        other = self._latest_rpm.get(3 - feedback.motor_id)
        if (reference_time is None or now - reference_time > FEEDBACK_STALE_S
                or other is None or now - other[1] > FEEDBACK_STALE_S):
            return  # Paused: only the reference time moves on.
        rpms = {motor_id: rpm for motor_id, (rpm, _) in self._latest_rpm.items()}
        left_rpm, right_rpm = wheel_rpm_from_motors(rpms, self._config_getter())
        self._pose = integrate(self._pose, left_rpm, right_rpm, now - reference_time,
                               self._config_getter())
        self._stamp_ns = self._node.get_clock().now().nanoseconds
        self._has_unpublished_step = True
        if self._paused_at is not None:
            self._node.get_logger().info(
                f"wheel odometry resumed after {now - self._paused_at:.1f} s")
            self._paused_at = None
        self._last_integrated_at = now

    def publish(self) -> None:
        """Publish the latest pose if it advanced. Never raises: it runs in the control timer."""
        try:
            self._publish_if_due(time.monotonic())
        except Exception as error:  # noqa: BLE001 - odometry must not stop the drive control
            self._node.get_logger().error(f"wheel odometry publish failed: {error!r}",
                                          throttle_duration_sec=1.0)

    def _publish_if_due(self, now: float) -> None:
        if self._last_integrated_at is None:
            return
        if now - self._last_integrated_at > FEEDBACK_STALE_S:
            self._pause(now)
            return
        if not self._has_unpublished_step:
            return
        self._has_unpublished_step = False
        if self._last_published_ns is not None and self._stamp_ns <= self._last_published_ns:
            if not self._is_clock_warned:
                self._is_clock_warned = True
                self._node.get_logger().warn("wheel odometry stamp did not advance; skipped")
            return
        self._last_published_ns = self._stamp_ns
        stamp = Time(sec=self._stamp_ns // NANOSECONDS_PER_SECOND,
                     nanosec=self._stamp_ns % NANOSECONDS_PER_SECOND)
        self._odom_publisher.publish(self._build_odometry(stamp))
        if self.is_tf_enabled:
            self._tf_broadcaster.sendTransform(self._build_transform(stamp))

    def _pause(self, now: float) -> None:
        if self._paused_at is not None:
            return
        self._paused_at = self._last_integrated_at
        stale_motor_id = min(MOTOR_IDS, key=lambda motor_id: self._latest_rpm.get(
            motor_id, (0, float("-inf")))[1])
        self._node.get_logger().warn(
            f"wheel odometry paused: no feedback from motor {stale_motor_id} "
            f"for {FEEDBACK_STALE_S} s")

    def _build_odometry(self, stamp) -> Odometry:
        pose = self._pose
        z, w = yaw_to_quaternion(pose.yaw)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = ODOM_FRAME
        message.child_frame_id = BASE_FRAME
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.twist.twist.linear.x = pose.linear_velocity
        message.twist.twist.angular.z = pose.angular_velocity
        linear_sigma, angular_sigma = twist_sigma(
            pose.linear_velocity, pose.angular_velocity, self._config_getter())
        message.pose.covariance = covariance_matrix(
            {"x": pose.variance_x, "y": pose.variance_y, "yaw": pose.variance_yaw})
        message.twist.covariance = covariance_matrix(
            {"x": linear_sigma ** 2, "yaw": angular_sigma ** 2})
        return message

    def _build_transform(self, stamp) -> TransformStamped:
        z, w = yaw_to_quaternion(self._pose.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = self._pose.x
        transform.transform.translation.y = self._pose.y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        return transform


def covariance_matrix(used_variances: dict[str, float]) -> list[float]:
    """Diagonal 6x6; components not given are marked unusable with a huge variance."""
    matrix = [0.0] * 36
    for name, index in DIAGONAL.items():
        matrix[index] = used_variances.get(name, UNUSED_COVARIANCE)
    return matrix
