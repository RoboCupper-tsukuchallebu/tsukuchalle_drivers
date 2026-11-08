"""Builds a fake /sys/class/power_supply in a directory (the kernel's file names and units)."""

from pathlib import Path

# The robot PC (X13) while discharging: 55 %, 24.59 of 43.99 Wh (design 48.02 Wh), 10.346 W.
X13_BATTERY = {
    "type": "Battery",
    "present": "1",
    "status": "Discharging",
    "capacity": "55",
    "energy_now": "24590000",
    "energy_full": "43990000",
    "energy_full_design": "48020000",
    "power_now": "10346000",
    "voltage_now": "10937000",
    "cycle_count": "55",
}
OFFLINE_ADAPTER = {"type": "Mains", "online": "0"}


def write_supply(root, name, attributes):
    directory = Path(root) / name
    directory.mkdir(parents=True, exist_ok=True)
    for attribute, value in attributes.items():
        (directory / attribute).write_text(f"{value}\n")


def make_x13(root, **overrides):
    """The X13 battery plus an offline adapter. A value of None removes that attribute."""
    attributes = {**X13_BATTERY, **overrides}
    write_supply(root, "BAT0", {key: value for key, value in attributes.items() if value is not None})
    write_supply(root, "AC", OFFLINE_ADAPTER)
