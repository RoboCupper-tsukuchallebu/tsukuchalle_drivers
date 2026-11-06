"""Drive two DDT motors from /cmd_vel.

Motors stay disabled until the ~/enable service is called with data: true or the pad's
enable button is pressed (the disable button or ~/enable with data: false stops them).
"""

import glob
import math
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
JOY_STALE_S = 0.5  # A longer silence on /joy means joy_node restarted or the pad was replugged.
# STATUS comes at 10 Hz, so a DISABLED seen right after the request may be older than it.
DISABLE_CONFIRM_S = 0.15
STATE_ARMING = 1
STATE_STOPPING = 3
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


def find_port(configured_port: str) -> str:
    if configured_port:
        return configured_port
    ports = [port for pattern in PORT_PATTERNS for port in glob.glob(pattern)]
    if len(ports) != 1:
        raise RuntimeError(f"set the port parameter: expected one board, found {ports}")
    return ports[0]


class LoggingBridge(Bridge):
    """Routes bridge events to the ROS logger instead of stdout."""

    def __init__(self, port: str, logger):
        self._logger = logger
        super().__init__(port)

    def on_status_changed(self, state: int, stop_reason: int) -> None:
        self._logger.info(f"bridge state={STATE_NAMES.get(state, state)} "
                          f"stop_reason={STOP_REASONS.get(stop_reason, stop_reason)}")

    def on_error(self, error_code: int) -> None:
        self._logger.error(f"bridge error: {ERROR_NAMES.get(error_code, error_code)}",
                           throttle_duration_sec=1.0)

    def on_feedback(self, feedback) -> None:
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

        self._bridge = LoggingBridge(find_port(self.get_parameter("port").value), self.get_logger())
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

    def stop_motors(self) -> None:
        self._bridge.send(TYPE_DISABLE)
        self._bridge.wait_for_state(STATE_DISABLED, STATE_WAIT_S)

    def close(self) -> None:
        self._bridge.close()

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
        is_enabling = self._bridge.state == STATE_ACTIVE or self._arming_deadline is not None
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
        return SetParametersResult(successful=True)

    def _on_cmd_vel(self, message: Twist) -> None:
        # Commands sent while disabled must not start the motors on enable.
        if (self._bridge.state != STATE_ACTIVE or self._disable_requested_at is not None
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
        if self._bridge.state == STATE_ACTIVE:
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
        logger.info(f"enable requested by joy button {button}")
        self._begin_arming()
        self._arming_button = button

    def _disable_by_pad(self, button: int) -> None:
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
        try:
            response.success = self._enable() if request.data else self._disable()
        except OSError as error:
            self._handle_lost_board(error)
            response.success = False
        response.message = (f"state={STATE_NAMES.get(self._bridge.state, 'unknown')} "
                            f"stop_reason={STOP_REASONS.get(self._bridge.stop_reason, 'unknown')}")
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
        self._finish_arming()
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
        elif now > self._arming_deadline:
            self._arming_deadline = None
            self.get_logger().warn(
                f"enable by joy button {self._arming_button} failed: "
                f"state={STATE_NAMES.get(self._bridge.state, 'unknown')} "
                f"stop_reason={STOP_REASONS.get(self._bridge.stop_reason, 'unknown')}")

    def _handle_lost_board(self, error: OSError) -> None:
        # The firmware's command timeout stops the motors if the board is still powered.
        self.get_logger().fatal(f"lost the board: {error}")
        rclpy.try_shutdown()

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
            self._bridge.poll(0.0)
            self._check_disabling(now)
            self._check_arming(now)
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
