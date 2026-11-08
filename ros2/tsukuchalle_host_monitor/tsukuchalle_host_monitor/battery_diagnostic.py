"""Turns a BatteryReading into the DiagnosticStatus "host: battery"."""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

STATUS_NAME = "host: battery"
UNKNOWN = "unknown"
DISCHARGING = "Discharging"
MINUTES_PER_HOUR = 60.0
NO_BATTERY_HARDWARE_ID = "none"


def _format_number(value, decimals):
    return UNKNOWN if value is None else f"{value:.{decimals}f}"


def _minutes_remaining(reading):
    """Minutes until empty at the present power; only while discharging."""
    if reading.status != DISCHARGING:
        return None
    if reading.energy_now_wh is None or not reading.power_w:
        return None
    return reading.energy_now_wh / reading.power_w * MINUTES_PER_HOUR


def _ac_text(ac_online):
    if ac_online is None:
        return UNKNOWN
    return "online" if ac_online else "offline"


def _level(reading, warn_percent, error_percent):
    if reading.status != DISCHARGING or reading.capacity_percent is None:
        return DiagnosticStatus.OK
    if reading.capacity_percent <= error_percent:
        return DiagnosticStatus.ERROR
    if reading.capacity_percent <= warn_percent:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.OK


def build_battery_status(reading, warn_percent, error_percent):
    """reading is None when no battery was found: STALE, because nothing can be said."""
    status = DiagnosticStatus(name=STATUS_NAME)
    if reading is None:
        status.level = DiagnosticStatus.STALE
        status.message = "no battery found"
        status.hardware_id = NO_BATTERY_HARDWARE_ID
        return status

    status.level = _level(reading, warn_percent, error_percent)
    status.hardware_id = reading.name
    status.message = f"{_format_number(reading.capacity_percent, 0)}% {reading.status or UNKNOWN}"
    minutes_remaining = _minutes_remaining(reading)
    values = (
        ("capacity_percent", _format_number(reading.capacity_percent, 0)),
        ("status", reading.status or UNKNOWN),
        ("ac", _ac_text(reading.ac_online)),
        ("energy_wh", _format_number(reading.energy_now_wh, 2)),
        ("energy_full_wh", _format_number(reading.energy_full_wh, 2)),
        ("energy_full_design_wh", _format_number(reading.energy_full_design_wh, 2)),
        ("power_w", _format_number(reading.power_w, 2)),
        ("time_remaining_min", _format_number(minutes_remaining, 0)),
        ("health_percent", _format_number(reading.health_percent, 1)),
        ("cycle_count", UNKNOWN if reading.cycle_count is None else str(reading.cycle_count)),
        ("voltage_v", _format_number(reading.voltage_v, 3)),
    )
    status.values = [KeyValue(key=key, value=value) for key, value in values]
    return status
