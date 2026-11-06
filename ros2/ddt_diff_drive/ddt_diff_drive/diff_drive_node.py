"""Drive two DDT motors from /cmd_vel.

Motors stay disabled until the ~/enable service is called with data: true.
"""

import glob
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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

        self._bridge = LoggingBridge(find_port(self.get_parameter("port").value), self.get_logger())
        self._requested = WheelTargets()
        self._output = WheelTargets()
        self._last_command_time = None
        self._last_control_time = time.monotonic()
        self._ignore_commands_until = 0.0

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
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

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        overrides = {parameter.name: parameter.value for parameter in parameters}
        if ("command_timeout_s" in overrides
                and not is_valid_command_timeout(overrides["command_timeout_s"])):
            return SetParametersResult(
                successful=False,
                reason=f"command_timeout_s must be in (0, {MAX_COMMAND_TIMEOUT_S}]")
        if "port" in overrides:
            return SetParametersResult(successful=False, reason="port cannot change at runtime")
        if self._bridge.state == STATE_ACTIVE and WIRING_PARAMETERS & overrides.keys():
            return SetParametersResult(successful=False,
                                       reason="disable the motors before changing wheel assignment")
        config = self._read_config(overrides)
        error = validation_error(config, MAX_VELOCITY_RPM)
        if error:
            return SetParametersResult(successful=False, reason=error)
        self._config = config
        self._command_timeout_s = overrides.get("command_timeout_s", self._command_timeout_s)
        return SetParametersResult(successful=True)

    def _on_cmd_vel(self, message: Twist) -> None:
        # Commands sent while disabled must not start the motors on enable.
        if self._bridge.state != STATE_ACTIVE or time.monotonic() < self._ignore_commands_until:
            return
        self._requested = twist_to_wheels(message.linear.x, message.angular.z, self._config)
        self._last_command_time = time.monotonic()

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
        self.stop_motors()
        return self._bridge.state == STATE_DISABLED

    def _enable(self) -> bool:
        if self._bridge.state == STATE_ACTIVE:
            return True  # Repeating enable while moving must not disturb the speed.
        self._output = WheelTargets()
        self._requested = WheelTargets()
        self._last_command_time = None
        self._bridge.send(TYPE_ENABLE)
        is_active = self._bridge.wait_for_state(STATE_ACTIVE, STATE_WAIT_S)
        self._last_control_time = time.monotonic()
        self._ignore_commands_until = self._last_control_time + ENABLE_SETTLE_S
        return is_active

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
            if self._bridge.state != STATE_ACTIVE:
                # Never resume at speed after the firmware has stopped on its own.
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
