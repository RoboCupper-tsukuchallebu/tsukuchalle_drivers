"""ROS-independent battery guard state machine."""
from dataclasses import dataclass


@dataclass(frozen=True)
class GuardActions:
    warning: bool
    alarm: bool = False
    critical: bool = False


class BatteryGuard:
    def __init__(self, warn_percent=5.0, critical_percent=3.0, hysteresis_percent=2.0,
                 alarm_period_s=30.0):
        if not 0 <= critical_percent <= warn_percent <= 100 or not 0 <= hysteresis_percent <= 100 or alarm_period_s <= 0:
            raise ValueError("invalid battery guard thresholds or alarm period")
        self.warn_percent = warn_percent
        self.critical_percent = critical_percent
        self.hysteresis_percent = hysteresis_percent
        self.alarm_period_s = alarm_period_s
        self.warning = False
        self.critical_armed = True
        self.last_alarm = None

    def update(self, discharging, capacity, ac_online, now):
        if ac_online is True:
            self.warning = False
            self.critical_armed = True
            self.last_alarm = None
            return GuardActions(False)
        if capacity is None or discharging is None:
            return GuardActions(self.warning)
        if capacity > self.warn_percent + self.hysteresis_percent:
            self.warning = False
            self.last_alarm = None
        if capacity > self.critical_percent + self.hysteresis_percent:
            self.critical_armed = True
        if not discharging:
            return GuardActions(self.warning)
        if capacity <= self.warn_percent:
            self.warning = True
        critical = capacity <= self.critical_percent and self.critical_armed
        if critical:
            self.critical_armed = False
        alarm = self.warning and (self.last_alarm is None or now - self.last_alarm >= self.alarm_period_s)
        if alarm:
            self.last_alarm = now
        return GuardActions(self.warning, alarm, critical)
