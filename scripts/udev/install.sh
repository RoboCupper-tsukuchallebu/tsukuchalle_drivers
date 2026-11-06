#!/bin/bash
# Install the udev rules on the robot PC. Run as root: sudo bash install.sh
# Verifies the rules, shows the diff against the installed file, then reloads udev and re-triggers tty devices only.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "usage: sudo bash $0   (must run as root; takes no arguments)" >&2
    exit 2
fi

scriptDirectory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sourceRules="${scriptDirectory}/70-tsukuchalle.rules"
installedRules="/etc/udev/rules.d/70-tsukuchalle.rules"

if ! command -v udevadm >/dev/null; then
    echo "verify unavailable: udevadm not found (needs systemd 254 or later)" >&2
    exit 1
fi
if ! udevadm verify "${sourceRules}"; then
    echo "udevadm verify failed: not installing ${sourceRules}" >&2
    exit 1
fi

installedForDiff="${installedRules}"
if [ ! -e "${installedRules}" ]; then
    installedForDiff=/dev/null
fi
diffStatus=0
diff -u "${installedForDiff}" "${sourceRules}" || diffStatus=$?
if [ "${diffStatus}" -eq 0 ]; then
    echo "already installed: ${installedRules}"
    exit 0
fi
if [ "${diffStatus}" -ne 1 ]; then
    echo "diff failed (status ${diffStatus}): not installing" >&2
    exit 1
fi

install -o root -g root -m 644 "${sourceRules}" "${installedRules}"
udevadm control --reload
# Only tty: a bare "udevadm trigger" also touches net (Velodyne USB LAN) and input (joy).
if udevadm trigger --help 2>&1 | grep -q -- --settle; then
    udevadm trigger --settle --subsystem-match=tty --action=change
else
    udevadm trigger --subsystem-match=tty --action=change
    udevadm settle --timeout=10
fi
ls -l /dev/ddt_board /dev/ma10p_nmea /dev/ma10p_info || true
