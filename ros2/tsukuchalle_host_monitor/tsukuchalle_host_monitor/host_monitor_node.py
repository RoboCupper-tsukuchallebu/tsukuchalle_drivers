"""Publish battery diagnostics and execute low-battery guard actions."""
import shlex
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from tsukuchalle_host_monitor.battery_diagnostic import DISCHARGING, build_battery_status
from tsukuchalle_host_monitor.battery_guard import BatteryGuard
from tsukuchalle_host_monitor.power_supply import read_battery

DIAGNOSTICS_TOPIC = "/diagnostics"
DIAGNOSTICS_QOS_DEPTH = 10
PUBLISH_PERIOD_S = 1.0
SERVICE_TIMEOUT_S = 2.0
DEFAULT_POWER_SUPPLY_ROOT = "/sys/class/power_supply"
DEFAULT_WARN_PERCENT = 20.0
DEFAULT_ERROR_PERCENT = 10.0


class HostMonitorNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("host_monitor_node", parameter_overrides=parameter_overrides)
        self._power_supply_root = self.declare_parameter("power_supply_root", DEFAULT_POWER_SUPPLY_ROOT).value
        self._warn_percent = self.declare_parameter("battery_warn_percent", DEFAULT_WARN_PERCENT).value
        self._error_percent = self.declare_parameter("battery_error_percent", DEFAULT_ERROR_PERCENT).value
        if not 0.0 <= self._error_percent <= self._warn_percent <= 100.0:
            raise ValueError("need 0 <= battery_error_percent <= battery_warn_percent <= 100")
        self._guard = BatteryGuard(
            self.declare_parameter("low_battery_warn_percent", 5.0).value,
            self.declare_parameter("critical_battery_percent", 3.0).value,
            self.declare_parameter("hysteresis_percent", 2.0).value,
            self.declare_parameter("alarm_period_s", 30.0).value,
        )
        self._alarm_command = self.declare_parameter("alarm_command", "").value
        pause_name = self.declare_parameter("mission_pause_service", "/lightnav/mission/pause").value
        save_name = self.declare_parameter("save_map_service", "/localization_node/save_map").value
        localization_name = self.declare_parameter("localization_node", "/localization_node").value.rstrip("/")
        self._pause_client = self.create_client(Trigger, pause_name)
        self._save_client = self.create_client(Trigger, save_name)
        self._mode_client = self.create_client(GetParameters, localization_name + "/get_parameters")
        self._pending = []
        self._alarm_process = None
        self._publisher = self.create_publisher(DiagnosticArray, DIAGNOSTICS_TOPIC, DIAGNOSTICS_QOS_DEPTH)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._warning_publisher = self.create_publisher(Bool, "/system/low_battery", qos)
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self.publish_diagnostics)
        self._service_timer = self.create_timer(0.1, self._advance_services)

    def _start_alarm(self):
        if not self._alarm_command or self._alarm_process is not None:
            return
        try:
            command = shlex.split(self._alarm_command)
            if command:
                self._alarm_process = subprocess.Popen(command, shell=False)
        except (OSError, ValueError) as error:
            self.get_logger().error(f"battery alarm failed to start: {error}")

    def _queue_service(self, label, client, request, callback=None):
        self._pending.append([label, client, request, callback, time.monotonic() + SERVICE_TIMEOUT_S, None])

    def _advance_services(self):
        now = time.monotonic()
        for pending in self._pending[:]:
            label, client, request, callback, deadline, future = pending
            try:
                if future is None and client.service_is_ready():
                    pending[5] = client.call_async(request)
                    pending[4] = now + SERVICE_TIMEOUT_S
                elif future is not None and future.done():
                    self._pending.remove(pending)
                    result = future.result()
                    if callback:
                        callback(result)
                    elif result.success:
                        self.get_logger().info(f"{label} succeeded: {result.message}")
                    else:
                        self.get_logger().warning(f"{label} failed: {result.message}")
                elif now >= deadline:
                    self._pending.remove(pending)
                    if future is not None:
                        client.remove_pending_request(future)
                    self.get_logger().warning(f"{label}: {'response timeout' if future else 'service unavailable'}")
            except Exception as error:
                if pending in self._pending:
                    self._pending.remove(pending)
                self.get_logger().error(f"{label} failed: {error}")

    def _handle_mode(self, result):
        if len(result.values) == 1 and result.values[0].string_value == "mapping":
            self._queue_service("save map", self._save_client, Trigger.Request())
        else:
            self.get_logger().info("map save skipped: localization mode is not confirmed as mapping")

    def publish_diagnostics(self):
        if self._alarm_process is not None:
            exit_code = self._alarm_process.poll()
            if exit_code is not None:
                if exit_code != 0:
                    self.get_logger().warning(f"battery alarm exited with code {exit_code}")
                self._alarm_process = None
        reading = read_battery(self._power_supply_root)
        discharging = None if reading is None or reading.status is None else reading.status == DISCHARGING
        actions = self._guard.update(discharging, None if reading is None else reading.capacity_percent,
                                     None if reading is None else reading.ac_online, time.monotonic())
        status = build_battery_status(reading, self._warn_percent, self._error_percent)
        if actions.warning and reading is not None and reading.capacity_percent is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = f"low battery: {reading.capacity_percent:.0f}%"
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._publisher.publish(message)
        self._warning_publisher.publish(Bool(data=actions.warning))
        if actions.alarm:
            self._start_alarm()
        if actions.critical:
            self._queue_service("mission pause", self._pause_client, Trigger.Request())
            request = GetParameters.Request()
            request.names = ["mode"]
            self._queue_service("localization mode", self._mode_client, request, self._handle_mode)


def main(args=None):
    rclpy.init(args=args)
    node = HostMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
