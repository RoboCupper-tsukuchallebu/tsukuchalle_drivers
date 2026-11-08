"""Reads the battery and the AC adapter from Linux's /sys/class/power_supply.

Units follow the kernel's power_supply ABI: energy in uWh, power in uW, voltage in uV,
charge in uAh, current in uA. Every attribute can be missing or unreadable; that is None.
"""

from dataclasses import dataclass
from pathlib import Path

MICRO_PER_UNIT = 1_000_000
BATTERY_TYPE = "Battery"
MAINS_TYPE = "Mains"
AC_NAME_PREFIX = "AC"
DEVICE_SCOPE = "Device"  # A peripheral's battery (mouse, headset), not the computer's.


@dataclass(frozen=True)
class BatteryReading:
    name: str
    status: str | None
    capacity_percent: float | None
    energy_now_wh: float | None
    energy_full_wh: float | None
    energy_full_design_wh: float | None
    power_w: float | None
    voltage_v: float | None
    cycle_count: int | None
    ac_online: bool | None

    @property
    def health_percent(self):
        """Full charge against the design capacity: how much the battery has aged."""
        if not self.energy_full_wh or not self.energy_full_design_wh:
            return None
        return self.energy_full_wh / self.energy_full_design_wh * 100.0


def _read_text(directory, attribute):
    try:
        return (directory / attribute).read_text().strip()
    except (OSError, UnicodeDecodeError):
        return None


def _read_integer(directory, attribute):
    text = _read_text(directory, attribute)
    try:
        return int(text) if text is not None else None
    except ValueError:
        return None


def _read_micro_unit(directory, attribute):
    value = _read_integer(directory, attribute)
    return value / MICRO_PER_UNIT if value is not None else None


def _read_energy_wh(directory, energy_attribute, charge_attribute, voltage_v):
    """Energy in Wh; a charge-based battery (uAh) is converted with the present voltage."""
    energy_wh = _read_micro_unit(directory, energy_attribute)
    if energy_wh is not None:
        return energy_wh
    charge_ah = _read_micro_unit(directory, charge_attribute)
    if charge_ah is None or voltage_v is None:
        return None
    return charge_ah * voltage_v


def _read_power_w(directory, voltage_v):
    power_w = _read_micro_unit(directory, "power_now")
    if power_w is None:
        current_a = _read_micro_unit(directory, "current_now")
        if current_a is None or voltage_v is None:
            return None
        power_w = current_a * voltage_v
    return abs(power_w)  # Some drivers report the discharge current as negative.


def _is_system_battery(directory):
    if _read_text(directory, "type") != BATTERY_TYPE:
        return False
    if _read_text(directory, "scope") == DEVICE_SCOPE:
        return False
    return _read_text(directory, "present") != "0"


def _is_mains(directory):
    return _read_text(directory, "type") == MAINS_TYPE or directory.name.startswith(AC_NAME_PREFIX)


def _directories(root):
    try:
        return sorted(path for path in root.iterdir() if path.is_dir())
    except OSError:
        return []


def read_ac_online(root):
    """True if any adapter is online, False if all are offline, None if there is none."""
    states = [_read_text(directory, "online") for directory in _directories(root) if _is_mains(directory)]
    known_states = [state for state in states if state is not None]
    if not known_states:
        return None
    return any(state == "1" for state in known_states)


def read_battery(root):
    """The first system battery under root (sorted by name), or None if there is none."""
    root = Path(root)
    for directory in _directories(root):
        if not _is_system_battery(directory):
            continue
        voltage_v = _read_micro_unit(directory, "voltage_now")
        capacity = _read_integer(directory, "capacity")
        return BatteryReading(
            name=directory.name,
            status=_read_text(directory, "status"),
            capacity_percent=float(capacity) if capacity is not None else None,
            energy_now_wh=_read_energy_wh(directory, "energy_now", "charge_now", voltage_v),
            energy_full_wh=_read_energy_wh(directory, "energy_full", "charge_full", voltage_v),
            energy_full_design_wh=_read_energy_wh(
                directory, "energy_full_design", "charge_full_design", voltage_v
            ),
            power_w=_read_power_w(directory, voltage_v),
            voltage_v=voltage_v,
            cycle_count=_read_integer(directory, "cycle_count"),
            ac_online=read_ac_online(root),
        )
    return None
