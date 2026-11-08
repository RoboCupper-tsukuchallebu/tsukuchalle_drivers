"""Publishes the robot PC's battery to /diagnostics at 1 Hz.

Read-only: it reads sysfs and publishes /diagnostics. It touches no drive topic.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tsukuchalle_host_monitor.battery_diagnostic import build_battery_status
from tsukuchalle_host_monitor.power_supply import read_battery

DIAGNOSTICS_TOPIC = "/diagnostics"
DIAGNOSTICS_QOS_DEPTH = 10
PUBLISH_PERIOD_S = 1.0
DEFAULT_POWER_SUPPLY_ROOT = "/sys/class/power_supply"
DEFAULT_WARN_PERCENT = 20.0
DEFAULT_ERROR_PERCENT = 10.0


class HostMonitorNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("host_monitor_node", parameter_overrides=parameter_overrides)
        self._power_supply_root = self.declare_parameter(
            "power_supply_root", DEFAULT_POWER_SUPPLY_ROOT
        ).value
        self._warn_percent = self.declare_parameter(
            "battery_warn_percent", DEFAULT_WARN_PERCENT
        ).value
        self._error_percent = self.declare_parameter(
            "battery_error_percent", DEFAULT_ERROR_PERCENT
        ).value
        if not 0.0 <= self._error_percent <= self._warn_percent <= 100.0:
            raise ValueError(
                "need 0 <= battery_error_percent <= battery_warn_percent <= 100, got "
                f"{self._error_percent} and {self._warn_percent}"
            )
        self._publisher = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, DIAGNOSTICS_QOS_DEPTH
        )
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self.publish_diagnostics)

    def publish_diagnostics(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [
            build_battery_status(
                read_battery(self._power_supply_root), self._warn_percent, self._error_percent
            )
        ]
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = HostMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C, or SIGTERM from docker stop.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
