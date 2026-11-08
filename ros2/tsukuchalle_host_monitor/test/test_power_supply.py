import tempfile
import unittest
from pathlib import Path

from fake_sysfs import X13_BATTERY, make_x13, write_supply
from tsukuchalle_host_monitor.power_supply import read_ac_online, read_battery


class PowerSupplyTest(unittest.TestCase):
    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self.root = Path(self._temporary.name)

    def test_x13_values_are_converted_from_micro_units(self):
        make_x13(self.root)

        reading = read_battery(self.root)

        self.assertEqual(reading.name, "BAT0")
        self.assertEqual(reading.status, "Discharging")
        self.assertEqual(reading.capacity_percent, 55.0)
        self.assertAlmostEqual(reading.energy_now_wh, 24.59)
        self.assertAlmostEqual(reading.energy_full_wh, 43.99)
        self.assertAlmostEqual(reading.energy_full_design_wh, 48.02)
        self.assertAlmostEqual(reading.power_w, 10.346)
        self.assertAlmostEqual(reading.voltage_v, 10.937)
        self.assertEqual(reading.cycle_count, 55)
        self.assertIs(reading.ac_online, False)
        self.assertAlmostEqual(reading.health_percent, 91.6077, places=3)

    def test_charge_based_battery_uses_charge_and_current_with_the_voltage(self):
        write_supply(
            self.root,
            "BAT0",
            {
                "type": "Battery",
                "status": "Not charging",
                "capacity": "79",
                "charge_now": "6710000",
                "charge_full": "8442000",
                "charge_full_design": "8442000",
                "current_now": "-1000000",
                "voltage_now": "11705000",
            },
        )

        reading = read_battery(self.root)

        self.assertAlmostEqual(reading.energy_now_wh, 6.71 * 11.705)
        self.assertAlmostEqual(reading.energy_full_wh, 8.442 * 11.705)
        self.assertAlmostEqual(reading.power_w, 11.705)  # |-1 A| * 11.705 V
        self.assertAlmostEqual(reading.health_percent, 100.0)

    def test_power_now_wins_over_current_and_voltage(self):
        make_x13(self.root, current_now="9999999")

        self.assertAlmostEqual(read_battery(self.root).power_w, 10.346)

    def test_missing_attributes_are_none(self):
        make_x13(
            self.root,
            capacity=None,
            energy_now=None,
            energy_full=None,
            energy_full_design=None,
            power_now=None,
            voltage_now=None,
            cycle_count=None,
            status=None,
        )

        reading = read_battery(self.root)

        self.assertEqual(reading.name, "BAT0")
        for value in (
            reading.status,
            reading.capacity_percent,
            reading.energy_now_wh,
            reading.energy_full_wh,
            reading.energy_full_design_wh,
            reading.power_w,
            reading.voltage_v,
            reading.cycle_count,
            reading.health_percent,
        ):
            self.assertIsNone(value)

    def test_charge_without_voltage_has_no_energy(self):
        write_supply(self.root, "BAT0", {"type": "Battery", "charge_now": "6710000"})

        self.assertIsNone(read_battery(self.root).energy_now_wh)

    def test_unparsable_value_is_none(self):
        make_x13(self.root, capacity="not a number", cycle_count="")

        reading = read_battery(self.root)

        self.assertIsNone(reading.capacity_percent)
        self.assertIsNone(reading.cycle_count)

    def test_unreadable_attribute_is_none(self):
        make_x13(self.root)
        (self.root / "BAT0" / "capacity").unlink()
        (self.root / "BAT0" / "capacity").mkdir()  # Reading a directory raises OSError.

        self.assertIsNone(read_battery(self.root).capacity_percent)

    def test_no_battery_is_none(self):
        write_supply(self.root, "AC", {"type": "Mains", "online": "1"})

        self.assertIsNone(read_battery(self.root))

    def test_missing_root_is_none(self):
        self.assertIsNone(read_battery(self.root / "does-not-exist"))

    def test_peripheral_and_absent_batteries_are_skipped(self):
        write_supply(self.root, "hid-0001-battery", {**X13_BATTERY, "scope": "Device"})
        write_supply(self.root, "BAT0", {**X13_BATTERY, "present": "0"})
        write_supply(self.root, "BAT1", {**X13_BATTERY, "capacity": "42"})

        reading = read_battery(self.root)

        self.assertEqual(reading.name, "BAT1")
        self.assertEqual(reading.capacity_percent, 42.0)

    def test_ac_online_is_true_if_any_adapter_is_online(self):
        write_supply(self.root, "AC", {"type": "Mains", "online": "0"})
        write_supply(self.root, "ADP1", {"type": "Mains", "online": "1"})

        self.assertIs(read_ac_online(self.root), True)

    def test_usb_supplies_are_not_adapters(self):
        write_supply(self.root, "ucsi-source-psy-USBC000:001", {"type": "USB", "online": "1"})

        self.assertIsNone(read_ac_online(self.root))

    def test_adapter_found_by_name_prefix_without_type(self):
        write_supply(self.root, "AC0", {"online": "1"})

        self.assertIs(read_ac_online(self.root), True)


if __name__ == "__main__":
    unittest.main()
