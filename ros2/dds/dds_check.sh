#!/bin/sh
# Placed in front of every container command (ENTRYPOINT ["/dds_check.sh", ...]). Stops the container when
# a setting that would otherwise fail silently is wrong; then `exec "$@"`. POSIX sh, and only `id`, `test`
# and `cat`: it also runs in the minimal tsukuchalle-velodyne image, which has no ROS and no bash wrapper.
#   - uid: shared-memory files in /dev/shm belong to the uid that made them (0644), so a process with another
#     uid still discovers its peers but receives 0 messages, with no error. TSUKUCHALLE_UID=0 is allowed.
#   - Fast DDS settings: without the profile, rmw_fastrtps adds a 512 KiB shared-memory transport (best-effort
#     point clouds lose frames); without SYSTEM_DEFAULT the discovery range falls back to SUBNET and reaches the LAN.
# TSUKUCHALLE_DDS_MODE=lan (to look from a Mac) drops both Fast DDS variables so the default SUBNET applies.
fail() {
  echo "dds_check: $*" >&2
  exit 1
}

if [ "$#" -eq 0 ]; then
  echo "usage: dds_check.sh <command> [arguments...]" >&2
  exit 2
fi

if [ -n "${TSUKUCHALLE_UID:-}" ] && [ "$(id -u)" != "$TSUKUCHALLE_UID" ]; then
  fail "running as uid $(id -u) but TSUKUCHALLE_UID=$TSUKUCHALLE_UID (set 'user:' in compose or --user)"
fi

if [ "${TSUKUCHALLE_DDS_MODE:-}" = lan ]; then
  echo "dds_check: TSUKUCHALLE_DDS_MODE=lan: Fast DDS profile removed, discovery range SUBNET (reaches the LAN)" >&2
  unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE
  exec "$@"
fi

profiles_file="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
if [ ! -f "$profiles_file" ] || [ ! -r "$profiles_file" ]; then
  fail "FASTRTPS_DEFAULT_PROFILES_FILE='$profiles_file' is not a readable file"
fi
profiles_text=$(cat "$profiles_file")
case "$profiles_text" in
  *'is_default_profile="true"'*) ;;
  *) fail "$profiles_file has no is_default_profile=\"true\" participant profile" ;;
esac
if [ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != SYSTEM_DEFAULT ]; then
  fail "ROS_AUTOMATIC_DISCOVERY_RANGE='${ROS_AUTOMATIC_DISCOVERY_RANGE:-}' must be SYSTEM_DEFAULT"
fi
exec "$@"
