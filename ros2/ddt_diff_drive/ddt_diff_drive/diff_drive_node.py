"""Drive two DDT motors from /cmd_vel.

Motors stay disabled until the ~/enable service is called with data: true or the pad's
enable button is pressed (the disable button or ~/enable with data: false stops them).

A USB disconnect does not end the node: it reopens the board and, if the motors were enabled,
enables them again once the deadman is released and the sticks are centered.
"""

import glob
import math
import termios
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from ddt_diff_drive.ddt_bridge import (
    ERROR_NAMES,
    MAX_VELOCITY_RPM,
    STATE_ACTIVE,
    STATE_DISABLED,
    STATE_NAMES,
    STOP_REASONS,
    TYPE_DISABLE,
    TYPE_ENABLE,
    Bridge,
)
from ddt_diff_drive.kinematics import (
    DriveConfig,
    WheelTargets,
    clamp_wheels,
    limit_rate,
    to_motor_rpm,
    twist_to_wheels,
    validation_error,
)
from ddt_diff_drive.odometry_publisher import OdometryPublisher
from ddt_diff_drive.telemetry import BridgeTelemetry

CONTROL_PERIOD_S = 0.02
MAX_CONTROL_STEP_S = 0.04  # A stalled timer must not turn into one large speed jump.
MAX_COMMAND_TIMEOUT_S = 5.0
# ponytail: time gate for commands queued while enable blocks; assumes the backlog drains
# within this window. A live publisher still moves the motors right after enable.
ENABLE_SETTLE_S = 0.1
# Losing /cmd_vel must stop the robot soon even with a gentle acceleration setting.
MIN_COMMAND_LOSS_DECELERATION_RPM_PER_S = 200.0
STATE_WAIT_S = 1.5
JOY_NEUTRAL_THRESHOLD = 0.1
STATUS_STALE_S = 0.35  # STATUS comes at 10 Hz; older than this, the cached state is not trusted.
JOY_STALE_S = 0.5  # A longer silence on /joy means joy_node restarted or the pad was replugged.
# STATUS comes at 10 Hz, so a DISABLED seen right after the request may be older than it.
DISABLE_CONFIRM_S = 0.15
RECONNECT_PERIOD_S = 0.5
STATE_ARMING = 1
STATE_STOPPING = 3
# Connection states, named as in the design of Issue #6. RECONNECTING and WAITING_STATUS have
# no usable board; WAITING_NEUTRAL and ARMING are the automatic re-enable after a reconnect
# (a pad-started ENABLE leaves the connection state at CONNECTED).
CONNECTED = "CONNECTED"
RECONNECTING = "RECONNECTING"
WAITING_STATUS = "WAITING_STATUS"
WAITING_NEUTRAL = "WAITING_NEUTRAL"
ARMING = "ARMING"
PORT_PATTERNS = ("/dev/cu.usbmodem*", "/dev/ttyACM*")
WIRING_PARAMETERS = {"left_motor_id", "left_sign", "right_sign"}
DRIVE_PARAMETERS = {
    "wheel_radius_m": 0.0935,
    "track_width_m": 0.26,
    "left_motor_id": 1,
    "left_sign": 1,
    "right_sign": -1,
    "max_wheel_rpm": 100.0,
    "max_acceleration_rpm_per_s": 100.0,
}
JOY_PARAMETERS = {
    "joy_enable_button": 7,
    "joy_disable_button": 6,
    "joy_deadman_button": 0,
    "joy_neutral_axes": [0, 1],
}


@dataclass(frozen=True)
class JoyConfig:
    enable_button: int
    disable_button: int
    deadman_button: int
    neutral_axes: tuple


def joy_validation_error(config: JoyConfig) -> str | None:
    buttons = (config.enable_button, config.disable_button, config.deadman_button)
    if any(button < -1 for button in buttons):
        return "joy button numbers must be -1 or more"
    if config.enable_button >= 0 and config.disable_button < 0:
        return "joy_disable_button is required while joy_enable_button is set"
    used_buttons = [button for button in buttons if button >= 0]
    if len(set(used_buttons)) != len(used_buttons):
        return "joy_enable_button, joy_disable_button and joy_deadman_button must differ"
    if any(axis < 0 for axis in config.neutral_axes):
        return "joy_neutral_axes must be 0 or more"
    return None


def is_pressed(buttons: tuple, index: int) -> bool:
    return 0 <= index < len(buttons) and buttons[index] != 0


def is_rising_edge(previous: tuple, current: tuple, index: int) -> bool:
    return is_pressed(current, index) and not is_pressed(previous, index)


def are_axes_neutral(axes, indexes: tuple) -> bool:
    # A missing axis cannot be shown to be at rest, so it counts as not neutral.
    return all(index < len(axes) and abs(axes[index]) <= JOY_NEUTRAL_THRESHOLD
               for index in indexes)


def is_valid_command_timeout(seconds: float) -> bool:
    return math.isfinite(seconds) and 0 < seconds <= MAX_COMMAND_TIMEOUT_S


def is_valid_reenable_wait(seconds: float) -> bool:
    return math.isfinite(seconds) and seconds >= 0


def find_port(configured_port: str) -> str:
    if configured_port:
        return configured_port
    ports = [port for pattern in PORT_PATTERNS for port in glob.glob(pattern)]
    if len(ports) != 1:
        raise RuntimeError(f"set the port parameter: expected one board, found {ports}")
    return ports[0]


class LoggingBridge(Bridge):
    """Routes bridge events to the ROS logger instead of stdout."""

    def __init__(self, port: str, logger, telemetry: BridgeTelemetry, odometry: OdometryPublisher):
        self._logger = logger
        self._telemetry = telemetry
        self._odometry = odometry
        self.last_status_time = None
        super().__init__(port)

    def _on_status(self, state: int, stop_reason: int) -> None:
        self.last_status_time = time.monotonic()
        super()._on_status(state, stop_reason)

    def on_status_changed(self, state: int, stop_reason: int) -> None:
        self._logger.info(f"bridge state={STATE_NAMES.get(state, state)} "
                          f"stop_reason={STOP_REASONS.get(stop_reason, stop_reason)}")

    def on_error(self, error_code: int) -> None:
        self._logger.error(f"bridge error: {ERROR_NAMES.get(error_code, error_code)}",
                           throttle_duration_sec=1.0)

    def on_feedback(self, feedback) -> None:
        received_at = time.monotonic()
        self._telemetry.record(feedback, received_at)
        self._odometry.on_feedback(feedback, received_at)
        if feedback.fault != 0:
            self._logger.error(f"motor {feedback.motor_id} fault 0x{feedback.fault:02X}",
                               throttle_duration_sec=1.0)


class DiffDriveNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("diff_drive_node", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("port", "")
        self.declare_parameter("command_timeout_s", 0.5)
        for name, default in DRIVE_PARAMETERS.items():
            self.declare_parameter(name, default)
        self._config = self._read_config({})
        error = validation_error(self._config, MAX_VELOCITY_RPM)
        if error:
            raise ValueError(error)
        self._command_timeout_s = self.get_parameter("command_timeout_s").value
        if not is_valid_command_timeout(self._command_timeout_s):
            raise ValueError(f"command_timeout_s must be in (0, {MAX_COMMAND_TIMEOUT_S}]")

        for name, default in JOY_PARAMETERS.items():
            self.declare_parameter(name, default)
        self._joy_config = self._read_joy_config({})
        error = joy_validation_error(self._joy_config)
        if error:
            raise ValueError(error)

        self.declare_parameter("auto_reenable_wait_s", 10.0)
        self._auto_reenable_wait_s = self.get_parameter("auto_reenable_wait_s").value
        if not is_valid_reenable_wait(self._auto_reenable_wait_s):
            raise ValueError("auto_reenable_wait_s must be 0 or more")

        # A board that is missing at startup is a wiring mistake, so it ends the node here.
        self._configured_port = self.get_parameter("port").value
        self._port = find_port(self._configured_port)
        self.declare_parameter("publish_tf", True)
        self._telemetry = BridgeTelemetry()
        self._odometry = OdometryPublisher(self, lambda: self._config)
        self._odometry.is_tf_enabled = self.get_parameter("publish_tf").value
        self._bridge = LoggingBridge(self._port, self.get_logger(), self._telemetry, self._odometry)
        self._connection = CONNECTED
        self._was_enabled = False  # The motors were on when the board was lost.
        self._disconnect_count = 0
        self._auto_reenable_count = 0
        self._lost_at = 0.0
        self._next_reconnect_at = 0.0
        self._status_deadline = 0.0
        self._reconnected_at = 0.0
        self._last_joy_axes = None
        self._requested = WheelTargets()
        self._output = WheelTargets()
        self._last_command_time = None
        self._last_control_time = time.monotonic()
        self._ignore_commands_until = 0.0
        self._previous_buttons = None  # The first Joy message has no previous one to compare.
        self._arming_deadline = None  # Set while waiting for ACTIVE after ENABLE was sent.
        self._arming_button = None
        self._disable_requested_at = None  # Set until DISABLED is seen after a disable request.
        self._last_joy_time = time.monotonic()

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Joy, "joy", self._on_joy, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_timer(CONTROL_PERIOD_S, self._on_control_timer)
        self.get_logger().info("ready; motors stay disabled until ~/enable is called")

    @property
    def _firmware_state(self):
        return None if self._bridge is None else self._bridge.state

    @property
    def _is_board_ready(self) -> bool:
        return self._connection in (CONNECTED, WAITING_NEUTRAL, ARMING)

    def _describe_state(self) -> str:
        stop_reason = 0 if self._bridge is None else self._bridge.stop_reason
        return (f"state={STATE_NAMES.get(self._firmware_state, 'unknown')} "
                f"stop_reason={STOP_REASONS.get(stop_reason, 'unknown')}")

    def stop_motors(self) -> None:
        if self._bridge is None:
            return
        self._bridge.send(TYPE_DISABLE)
        self._bridge.wait_for_state(STATE_DISABLED, STATE_WAIT_S)

    def close(self) -> None:
        self._close_bridge()

    def _close_bridge(self) -> None:
        bridge, self._bridge = self._bridge, None
        if bridge is None:
            return
        try:
            bridge.close()
        except OSError:
            pass  # The descriptor of an unplugged board may already be gone.

    def _read_config(self, overrides: dict) -> DriveConfig:
        values = {name: overrides.get(name, self.get_parameter(name).value)
                  for name in DRIVE_PARAMETERS}
        return DriveConfig(**values)

    def _read_joy_config(self, overrides: dict) -> JoyConfig:
        values = {name: overrides.get(name, self.get_parameter(name).value)
                  for name in JOY_PARAMETERS}
        return JoyConfig(values["joy_enable_button"], values["joy_disable_button"],
                         values["joy_deadman_button"], tuple(values["joy_neutral_axes"]))

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        overrides = {parameter.name: parameter.value for parameter in parameters}
        if ("command_timeout_s" in overrides
                and not is_valid_command_timeout(overrides["command_timeout_s"])):
            return SetParametersResult(
                successful=False,
                reason=f"command_timeout_s must be in (0, {MAX_COMMAND_TIMEOUT_S}]")
        if "port" in overrides:
            return SetParametersResult(successful=False, reason="port cannot change at runtime")
        if "publish_tf" in overrides and not isinstance(overrides["publish_tf"], bool):
            return SetParametersResult(successful=False, reason="publish_tf must be a bool")
        if ("auto_reenable_wait_s" in overrides
                and not is_valid_reenable_wait(overrides["auto_reenable_wait_s"])):
            return SetParametersResult(successful=False,
                                       reason="auto_reenable_wait_s must be 0 or more")
        is_enabling = (self._firmware_state == STATE_ACTIVE or self._arming_deadline is not None
                       or self._was_enabled)
        if is_enabling and WIRING_PARAMETERS & overrides.keys():
            return SetParametersResult(successful=False,
                                       reason="disable the motors before changing wheel assignment")
        config = self._read_config(overrides)
        error = validation_error(config, MAX_VELOCITY_RPM)
        if error:
            return SetParametersResult(successful=False, reason=error)
        joy_config = self._read_joy_config(overrides)
        error = joy_validation_error(joy_config)
        if error:
            return SetParametersResult(successful=False, reason=error)
        self._config = config
        self._joy_config = joy_config
        self._command_timeout_s = overrides.get("command_timeout_s", self._command_timeout_s)
        self._auto_reenable_wait_s = overrides.get("auto_reenable_wait_s",
                                                   self._auto_reenable_wait_s)
        self._odometry.is_tf_enabled = overrides.get("publish_tf", self._odometry.is_tf_enabled)
        return SetParametersResult(successful=True)

    def _on_cmd_vel(self, message: Twist) -> None:
        # Commands sent while disabled must not start the motors on enable.
        if (self._firmware_state != STATE_ACTIVE or self._disable_requested_at is not None
                or time.monotonic() < self._ignore_commands_until):
            return
        self._requested = twist_to_wheels(message.linear.x, message.angular.z, self._config)
        self._last_command_time = time.monotonic()

    def _on_joy(self, message: Joy) -> None:
        buttons = tuple(message.buttons)
        now = time.monotonic()
        previous, self._previous_buttons = self._previous_buttons, buttons
        is_after_silence = now - self._last_joy_time >= JOY_STALE_S
        self._last_joy_time = now
        self._last_joy_axes = tuple(message.axes)
        if previous is None:
            return
        config = self._joy_config
        try:
            if is_rising_edge(previous, buttons, config.disable_button):
                self._disable_by_pad(config.disable_button)
            # The first message after a silence is not an edge: Start may have been held
            # through a joy_node restart. Stopping stays allowed on it (above).
            elif not is_after_silence and is_rising_edge(previous, buttons, config.enable_button):
                self._enable_by_pad(config.enable_button, buttons, message.axes)
        except OSError as error:
            self._handle_lost_board(error)

    def _enable_by_pad(self, button: int, buttons: tuple, axes) -> None:
        # Starting the motors needs a calm pad; stopping them never does (see _disable_by_pad).
        logger = self.get_logger()
        if not self._is_board_ready:
            logger.warn("enable ignored: board disconnected", throttle_duration_sec=1.0)
            return
        if self._firmware_state == STATE_ACTIVE:
            return
        if self._arming_deadline is not None:
            logger.warn("enable ignored: already arming", throttle_duration_sec=1.0)
            return
        if self._disable_requested_at is not None:
            logger.warn("enable ignored: disable in progress", throttle_duration_sec=1.0)
            return
        if is_pressed(buttons, self._joy_config.disable_button):
            logger.warn("enable ignored: the disable button is held", throttle_duration_sec=1.0)
            return
        if (is_pressed(buttons, self._joy_config.deadman_button)
                or not are_axes_neutral(axes, self._joy_config.neutral_axes)):
            logger.warn("enable ignored: release the sticks and the deadman button first",
                        throttle_duration_sec=1.0)
            return
        self._supersede_auto_reenable(f"joy button {button}")
        logger.info(f"enable requested by joy button {button}")
        self._begin_arming()
        self._arming_button = button

    def _disable_by_pad(self, button: int) -> None:
        self._cancel_auto_reenable(f"joy button {button}")
        if not self._is_board_ready:
            return  # Nothing is running and there is nobody to send to.
        self._request_disable()
        self.get_logger().info(f"disabled by joy button {button}")

    def _request_disable(self) -> None:
        # Does not wait for DISABLED: the control timer holds the output at zero, drops
        # /cmd_vel, resends DISABLE and reports a failure until DISABLED is seen. The
        # firmware may be mid-ARMING, or the first frame may be lost.
        self._arming_deadline = None
        self._requested = WheelTargets()
        self._output = WheelTargets()
        self._last_command_time = None
        self._disable_requested_at = time.monotonic()
        self._bridge.send(TYPE_DISABLE)

    def _check_disabling(self, now: float) -> None:
        if self._disable_requested_at is None:
            return
        state = self._bridge.state
        if state == STATE_DISABLED:
            if now - self._disable_requested_at >= DISABLE_CONFIRM_S:
                self._disable_requested_at = None
            return
        if state in (STATE_ARMING, STATE_ACTIVE):
            self._bridge.send(TYPE_DISABLE)
        if now - self._disable_requested_at > STATE_WAIT_S:
            self.get_logger().error(
                f"disable not confirmed: state={STATE_NAMES.get(state, 'unknown')} "
                f"stop_reason={STOP_REASONS.get(self._bridge.stop_reason, 'unknown')}",
                throttle_duration_sec=1.0)

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if not self._is_board_ready:
            return self._answer_while_disconnected(request, response)
        try:
            if request.data:
                self._supersede_auto_reenable("~/enable")
                response.success = self._enable()
            else:
                self._cancel_auto_reenable("~/enable")
                response.success = self._disable()
        except OSError as error:
            self._handle_lost_board(error)
            response.success = False
        response.message = self._describe_state()
        return response

    def _answer_while_disconnected(self, request, response):
        if request.data:
            self.get_logger().warn("enable ignored: board disconnected", throttle_duration_sec=1.0)
            response.success = False
            response.message = "board disconnected"
        else:
            self._cancel_auto_reenable("~/enable")
            response.success = True  # Stopping succeeds: nothing is running.
            response.message = "board disconnected; auto re-enable cancelled"
        return response

    def _disable(self) -> bool:
        self._request_disable()
        self._bridge.wait_for_state(STATE_DISABLED, STATE_WAIT_S)
        return self._bridge.state == STATE_DISABLED

    def _enable(self) -> bool:
        if self._bridge.state == STATE_ACTIVE:
            return True  # Repeating enable while moving must not disturb the speed.
        if self._arming_deadline is None:
            self._begin_arming()
        is_active = self._bridge.wait_for_state(STATE_ACTIVE, STATE_WAIT_S)
        if self._connection != ARMING:
            self._finish_arming()
        elif is_active:
            self._finish_arming()  # Start the settle window now, not at the next timer tick.
            self._finish_rearming()
        return is_active

    def _begin_arming(self) -> None:
        self._disable_requested_at = None
        self._output = WheelTargets()
        self._requested = WheelTargets()
        self._last_command_time = None
        self._bridge.send(TYPE_ENABLE)
        self._arming_deadline = time.monotonic() + STATE_WAIT_S

    def _finish_arming(self) -> None:
        self._arming_deadline = None
        self._last_control_time = time.monotonic()
        self._ignore_commands_until = self._last_control_time + ENABLE_SETTLE_S

    def _check_arming(self, now: float) -> None:
        if self._arming_deadline is None:
            return
        if self._bridge.state == STATE_ACTIVE:
            self._finish_arming()
            if self._connection == ARMING:
                self._finish_rearming()
        elif now > self._arming_deadline and self._connection == ARMING:
            self._arming_deadline = None
            self._connection = CONNECTED
            self._was_enabled = False
            self.get_logger().error(f"auto re-enable failed: {self._describe_state()}")
        elif now > self._arming_deadline:
            self._arming_deadline = None
            self.get_logger().warn(
                f"enable by joy button {self._arming_button} failed: "
                f"state={STATE_NAMES.get(self._bridge.state, 'unknown')} "
                f"stop_reason={STOP_REASONS.get(self._bridge.stop_reason, 'unknown')}")

    def _handle_lost_board(self, error: OSError) -> None:
        # USB unplugged or the board reset. If it is still powered, the firmware's command
        # timeout stops the motors. No retry limit (user decision): every loss is logged.
        if not self._is_board_ready:
            return
        was_enabled = self._was_enabled or self._is_enabled_by_last_status(time.monotonic())
        state_name = STATE_NAMES.get(self._firmware_state, "unknown")
        self._close_bridge()
        self._was_enabled = was_enabled
        self._disconnect_count += 1
        self._connection = RECONNECTING
        self._lost_at = time.monotonic()
        self._next_reconnect_at = self._lost_at + RECONNECT_PERIOD_S
        self._requested = WheelTargets()
        self._output = WheelTargets()
        self._last_command_time = None
        self._arming_deadline = None
        self._disable_requested_at = None
        self.get_logger().error(f"lost the board: {error}; reconnecting "
                                f"(was {state_name}, disconnect #{self._disconnect_count} "
                                f"since start)")

    def _is_enabled_by_last_status(self, now: float) -> bool:
        # Judged from the last STATUS only: a firmware that stopped itself (feedback lost,
        # arming timeout, ...) must not be switched on again by a reconnect.
        bridge = self._bridge
        if self._disable_requested_at is not None or bridge.stop_reason != 0:
            return False
        if bridge.last_status_time is None or now - bridge.last_status_time > STATUS_STALE_S:
            return False
        return bridge.state in (STATE_ARMING, STATE_ACTIVE) or (
            bridge.state == STATE_DISABLED and self._arming_deadline is not None)

    def _advance_reconnect(self, now: float) -> None:
        if self._connection == RECONNECTING:
            self._try_open_board(now)
            return
        try:
            self._bridge.poll(0.0)
        except OSError as error:
            self._abort_reconnect(now, str(error), throttle_duration_sec=5.0)
            return
        if self._bridge.state == STATE_DISABLED:
            self._finish_reconnect(now)
        elif now > self._status_deadline:
            self._abort_reconnect(now, f"no STATUS from {self._port} within {STATE_WAIT_S} s")

    def _try_open_board(self, now: float) -> None:
        if now < self._next_reconnect_at:
            return
        self._next_reconnect_at = now + RECONNECT_PERIOD_S
        try:
            port = find_port(self._configured_port)
            self._bridge = LoggingBridge(port, self.get_logger(), self._telemetry, self._odometry)
        except (OSError, RuntimeError, termios.error) as error:
            self.get_logger().warn(f"reconnect failed: {error}", throttle_duration_sec=5.0)
            return
        self._port = port
        self._connection = WAITING_STATUS
        self._status_deadline = now + STATE_WAIT_S

    def _abort_reconnect(self, now: float, reason: str, throttle_duration_sec=None) -> None:
        self._close_bridge()
        self._connection = RECONNECTING
        self._next_reconnect_at = now + RECONNECT_PERIOD_S
        self.get_logger().warn(f"reconnect failed: {reason}",
                               throttle_duration_sec=throttle_duration_sec)

    def _finish_reconnect(self, now: float) -> None:
        self.get_logger().warn(f"board reconnected on {self._port} after "
                               f"{now - self._lost_at:.1f} s "
                               f"(disconnect #{self._disconnect_count} since start)")
        if self._was_enabled:
            self._connection = WAITING_NEUTRAL
            self._reconnected_at = now
        else:
            self._connection = CONNECTED

    def _is_pad_neutral(self, now: float) -> bool:
        config = self._joy_config
        if self._last_joy_axes is not None and now - self._last_joy_time < JOY_STALE_S:
            # Even with the pad's enable button off, teleop_twist_joy still drives from it.
            return (not is_pressed(self._previous_buttons, config.deadman_button)
                    and are_axes_neutral(self._last_joy_axes, config.neutral_axes))
        return config.enable_button < 0  # No pad in use; with one, a silent pad is not at rest.

    def _advance_waiting_neutral(self, now: float) -> None:
        if self._connection != WAITING_NEUTRAL:
            return
        if self._is_pad_neutral(now):
            self._connection = ARMING
            self._begin_arming()
        elif now > self._reconnected_at + self._auto_reenable_wait_s:  # Live: edits apply.
            self._connection = CONNECTED
            self._was_enabled = False
            self.get_logger().error(
                "auto re-enable abandoned: sticks or deadman not neutral within "
                f"{self._auto_reenable_wait_s} s; staying DISABLED until enabled by hand")

    def _finish_rearming(self) -> None:
        self._connection = CONNECTED
        self._was_enabled = False
        self._auto_reenable_count += 1
        self.get_logger().warn("auto re-enabled after board reconnect "
                               f"(total {self._auto_reenable_count} since start)")

    def _cancel_auto_reenable(self, source: str) -> None:
        if not self._was_enabled:
            return
        self._was_enabled = False
        if self._connection in (WAITING_NEUTRAL, ARMING):
            self._connection = CONNECTED
        self.get_logger().info(f"auto re-enable cancelled by {source}")

    def _supersede_auto_reenable(self, source: str) -> None:
        if self._connection != WAITING_NEUTRAL:
            return
        self._connection = CONNECTED
        self._was_enabled = False
        self.get_logger().info(f"auto re-enable superseded by {source}")

    def _is_command_fresh(self, now: float) -> bool:
        return (self._last_command_time is not None
                and now - self._last_command_time < self._command_timeout_s)

    def _current_target(self, now: float) -> WheelTargets:
        if not self._is_command_fresh(now):
            return WheelTargets()
        # Re-clamp so that lowering max_wheel_rpm also applies to the held command.
        return clamp_wheels(self._requested, self._config.max_wheel_rpm)

    def _on_control_timer(self) -> None:
        now = time.monotonic()
        elapsed_s = min(now - self._last_control_time, MAX_CONTROL_STEP_S)
        self._last_control_time = now
        try:
            if not self._is_board_ready:
                self._advance_reconnect(now)
                return
            self._bridge.poll(0.0)
            self._check_disabling(now)
            self._check_arming(now)
            self._advance_waiting_neutral(now)
            if self._bridge.state != STATE_ACTIVE or self._disable_requested_at is not None:
                # Never resume at speed after the firmware has stopped on its own. Right after
                # DISABLE the cached ACTIVE is stale for up to a STATUS period, and a
                # SET_VELOCITY to the stopping firmware is answered with "invalid state".
                self._output = WheelTargets()
                return
            rate_rpm_per_s = self._config.max_acceleration_rpm_per_s
            if not self._is_command_fresh(now):
                rate_rpm_per_s = max(rate_rpm_per_s, MIN_COMMAND_LOSS_DECELERATION_RPM_PER_S)
            self._output = limit_rate(self._output, self._current_target(now),
                                      rate_rpm_per_s * elapsed_s)
            self._bridge.set_velocity(*to_motor_rpm(self._output, self._config))
        except OSError as error:
            self._handle_lost_board(error)
        finally:
            self._odometry.publish()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = DiffDriveNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.stop_motors()
            except OSError as error:
                print(f"could not confirm stop: {error}")
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
