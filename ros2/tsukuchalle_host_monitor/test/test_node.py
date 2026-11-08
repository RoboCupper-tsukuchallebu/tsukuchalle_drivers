"""Runs the real node against a fake sysfs and reads what it publishes on /diagnostics."""

import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from fake_sysfs import make_x13
from tsukuchalle_host_monitor.host_monitor_node import DIAGNOSTICS_TOPIC, HostMonitorNode

RECEIVE_TIMEOUT_S = 10.0
QOS_DEPTH = 10


class HostMonitorNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self.root = Path(self._temporary.name)

    def receive_first_message(self, **parameters):
        node = create_node(self.root, parameters)
        listener = rclpy.create_node("host_monitor_listener")
        received = []
        listener.create_subscription(DiagnosticArray, DIAGNOSTICS_TOPIC, received.append, QOS_DEPTH)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(listener)
        try:
            deadline = time.monotonic() + RECEIVE_TIMEOUT_S
            while not received and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
        finally:
            executor.shutdown()
            node.destroy_node()
            listener.destroy_node()
        self.assertTrue(received, "no /diagnostics message within the timeout")
        return received[0]

    def test_publishes_the_battery_status(self):
        make_x13(self.root, capacity="8")

        message = self.receive_first_message()

        self.assertEqual(len(message.status), 1)
        self.assertEqual(message.status[0].name, "host: battery")
        self.assertEqual(message.status[0].hardware_id, "BAT0")
        self.assertEqual(message.status[0].level, DiagnosticStatus.ERROR)

    def test_publishes_stale_without_a_battery(self):
        message = self.receive_first_message()

        self.assertEqual(message.status[0].level, DiagnosticStatus.STALE)

    def test_thresholds_come_from_parameters(self):
        make_x13(self.root, capacity="30")

        message = self.receive_first_message(battery_warn_percent=40.0)

        self.assertEqual(message.status[0].level, DiagnosticStatus.WARN)

    def test_inconsistent_thresholds_are_rejected(self):
        with self.assertRaises(ValueError):
            create_node(
                self.root, {"battery_warn_percent": 5.0, "battery_error_percent": 10.0}
            )


def create_node(root, parameters):
    overrides = [Parameter("power_supply_root", value=str(root))]
    overrides += [Parameter(name, value=value) for name, value in parameters.items()]
    return HostMonitorNode(parameter_overrides=overrides)


if __name__ == "__main__":
    unittest.main()
