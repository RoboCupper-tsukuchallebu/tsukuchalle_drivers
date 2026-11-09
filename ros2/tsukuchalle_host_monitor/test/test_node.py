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


class BatteryGuardNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.monitor = create_node(self.root, {"alarm_command": "/nonexistent-battery-alarm"})
        self.listener = rclpy.create_node("guard_test_listener")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.monitor)
        self.executor.add_node(self.listener)
        self.diagnostics = []
        self.warnings = []
        self.listener.create_subscription(DiagnosticArray, DIAGNOSTICS_TOPIC, self.diagnostics.append, QOS_DEPTH)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.listener.create_subscription(Bool, "/system/low_battery", self.warnings.append, qos)
        self.addCleanup(self.monitor.destroy_node)
        self.addCleanup(self.listener.destroy_node)
        self.addCleanup(self.executor.shutdown)

    def spin_until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
        self.assertTrue(predicate())

    def test_warning_and_ac_clear(self):
        from fake_sysfs import write_supply
        make_x13(self.root, capacity="5")
        self.spin_until(lambda: self.diagnostics and self.warnings and self.warnings[-1].data)
        self.assertEqual(self.diagnostics[-1].status[0].level, DiagnosticStatus.ERROR)
        self.assertIn("5%", self.diagnostics[-1].status[0].message)
        write_supply(self.root, "AC", {"type": "Mains", "online": "1"})
        self.spin_until(lambda: self.warnings and not self.warnings[-1].data)

    def test_critical_calls_each_service_once(self):
        from std_srvs.srv import Trigger
        make_x13(self.root, capacity="3")
        localization = rclpy.create_node("localization_node")
        localization.declare_parameter("mode", "mapping")
        self.executor.add_node(localization)
        self.addCleanup(localization.destroy_node)
        calls = []
        def respond(label):
            def callback(request, response):
                calls.append(label)
                response.success = True
                return response
            return callback
        self.listener.create_service(Trigger, "/lightnav/mission/pause", respond("pause"))
        localization.create_service(Trigger, "/localization_node/save_map", respond("save"))
        self.spin_until(lambda: len(calls) >= 2)
        end = time.monotonic() + 2.2
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.1)
        self.assertCountEqual(calls, ["pause", "save"])

    def test_missing_services_do_not_stop_node(self):
        make_x13(self.root, capacity="3")
        self.spin_until(lambda: len(self.diagnostics) >= 4)
        self.assertTrue(self.warnings[-1].data)
