import dataclasses
import tempfile
import unittest
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticStatus

from fake_sysfs import make_x13
from tsukuchalle_host_monitor.battery_diagnostic import STATUS_NAME, build_battery_status
from tsukuchalle_host_monitor.power_supply import read_battery

WARN_PERCENT = 20.0
ERROR_PERCENT = 10.0


class BatteryDiagnosticTest(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self.root = Path(self._temporary.name)

    def build(self, **overrides):
        make_x13(self.root, **overrides)
        return build_battery_status(read_battery(self.root), WARN_PERCENT, ERROR_PERCENT)

    @staticmethod
    def values_of(status):
        return {item.key: item.value for item in status.values}

    def test_x13_discharging_is_ok_with_every_value(self):
        status = self.build()

        self.assertEqual(status.name, STATUS_NAME)
        self.assertEqual(status.name, "host: battery")
        self.assertEqual(status.hardware_id, "BAT0")
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "55% Discharging")
        self.assertEqual(
            self.values_of(status),
            {
                "capacity_percent": "55",
                "status": "Discharging",
                "ac": "offline",
                "energy_wh": "24.59",
                "energy_full_wh": "43.99",
                "energy_full_design_wh": "48.02",
                "power_w": "10.35",
                "time_remaining_min": "143",  # 24.59 Wh / 10.346 W = 2.377 h
                "health_percent": "91.6",
                "cycle_count": "55",
                "voltage_v": "10.937",
            },
        )

    def test_level_thresholds_while_discharging(self):
        cases = (
            ("21", DiagnosticStatus.OK),
            ("20", DiagnosticStatus.WARN),
            ("11", DiagnosticStatus.WARN),
            ("10", DiagnosticStatus.ERROR),
            ("0", DiagnosticStatus.ERROR),
        )
        for capacity, expected_level in cases:
            with self.subTest(capacity=capacity):
                self.assertEqual(self.build(capacity=capacity).level, expected_level)

    def test_low_capacity_is_ok_while_charging_or_full(self):
        for charging_status in ("Charging", "Full", "Not charging"):
            with self.subTest(status=charging_status):
                status = self.build(capacity="5", status=charging_status)
                self.assertEqual(status.level, DiagnosticStatus.OK)

    def test_charging_has_no_remaining_time(self):
        status = self.build(status="Charging")

        self.assertEqual(self.values_of(status)["time_remaining_min"], "unknown")
        self.assertEqual(status.message, "55% Charging")

    def test_no_battery_is_stale_with_a_message(self):
        status = build_battery_status(None, WARN_PERCENT, ERROR_PERCENT)

        self.assertEqual(status.name, "host: battery")
        self.assertEqual(status.level, DiagnosticStatus.STALE)
        self.assertEqual(status.message, "no battery found")
        self.assertEqual(list(status.values), [])

    def test_missing_values_are_unknown(self):
        status = self.build(
            capacity=None,
            status=None,
            energy_now=None,
            energy_full=None,
            energy_full_design=None,
            power_now=None,
            voltage_now=None,
            cycle_count=None,
        )

        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "unknown% unknown")
        values = self.values_of(status)
        self.assertEqual(values["ac"], "offline")
        for key, value in values.items():
            if key != "ac":
                self.assertEqual(value, "unknown", key)

    def test_unknown_adapter_is_unknown(self):
        reading = dataclasses.replace(self._x13_reading(), ac_online=None)

        status = build_battery_status(reading, WARN_PERCENT, ERROR_PERCENT)

        self.assertEqual(self.values_of(status)["ac"], "unknown")

    def test_zero_power_has_no_remaining_time(self):
        status = self.build(power_now="0")

        self.assertEqual(self.values_of(status)["time_remaining_min"], "unknown")

    def test_thresholds_are_parameters(self):
        reading = self._x13_reading()

        self.assertEqual(build_battery_status(reading, 60.0, 50.0).level, DiagnosticStatus.WARN)
        self.assertEqual(build_battery_status(reading, 60.0, 55.0).level, DiagnosticStatus.ERROR)

    def _x13_reading(self):
        make_x13(self.root)
        return read_battery(self.root)


if __name__ == "__main__":
    unittest.main()
