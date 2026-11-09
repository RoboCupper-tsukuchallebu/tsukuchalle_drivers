import unittest

from tsukuchalle_host_monitor.battery_guard import BatteryGuard


class BatteryGuardTest(unittest.TestCase):
    def test_boundaries(self):
        for capacity, warning, critical in [(5.1, False, False), (5, True, False), (4.9, True, False), (3.1, True, False), (3, True, True), (2.9, True, True)]:
            with self.subTest(capacity=capacity):
                action = BatteryGuard().update(True, capacity, False, 0)
                self.assertEqual((action.warning, action.critical), (warning, critical))

    def test_alarm_and_rearm(self):
        guard = BatteryGuard()
        self.assertTrue(guard.update(True, 3, False, 0).critical)
        self.assertFalse(guard.update(True, 3, False, 1).critical)
        self.assertFalse(guard.update(True, 3, False, 29).alarm)
        self.assertTrue(guard.update(True, 3, False, 30).alarm)
        self.assertFalse(guard.update(True, 5, False, 31).critical)
        guard.update(True, 5.1, False, 32)
        self.assertTrue(guard.update(True, 3, False, 33).critical)
        self.assertTrue(guard.update(True, 7, False, 34).warning)
        self.assertFalse(guard.update(True, 7.1, False, 35).warning)
        guard.update(True, 3, False, 36)
        self.assertFalse(guard.update(True, 3, True, 37).warning)
        self.assertTrue(guard.update(True, 3, False, 38).critical)

    def test_unknown_and_ac(self):
        for discharging, capacity, ac in [(None, None, None), (True, None, False), (False, 1, False), (True, 1, True)]:
            action = BatteryGuard().update(discharging, capacity, ac, 0)
            self.assertFalse(action.warning)
            self.assertFalse(action.critical)
            self.assertFalse(action.alarm)
