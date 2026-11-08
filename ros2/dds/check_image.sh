#!/bin/bash
# Static checks of the DDS files baked into a built image, run on the CI host (not inside the image):
#   ros2/dds/check_image.sh <image>
# The ENV and ENTRYPOINT of ros2/Dockerfile, ros2/tools/Dockerfile and ros2/velodyne/Dockerfile (runtime)
# must agree, and the two copies of fastdds_profiles.xml / dds_check.sh must be byte-identical to the
# repository files (the same sha256 other repositories compare against this repository's develop).
set -euo pipefail

image=${1:?usage: check_image.sh <image>}
script_directory=$(cd "$(dirname "$0")" && pwd)
failure_count=0
fail() {
  echo "FAIL: $image: $*"
  failure_count=$((failure_count + 1))
}

environment=$(docker image inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$image")
for required in ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
  FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml ROS_LOG_DIR=/tmp/ros_log ROS_HOME=/tmp/ros_home; do
  grep -qxF "$required" <<< "$environment" || fail "ENV $required is missing"
done
for forbidden in FASTDDS_BUILTIN_TRANSPORTS ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; do
  ! grep -q "^$forbidden" <<< "$environment" || fail "ENV $forbidden must not be set"
done

entrypoint_first=$(docker image inspect --format '{{index .Config.Entrypoint 0}}' "$image")
[[ $entrypoint_first == /dds_check.sh ]] || fail "ENTRYPOINT starts with '$entrypoint_first', not /dds_check.sh"

expected=$(cd "$script_directory" && sha256sum fastdds_profiles.xml dds_check.sh | cut -d' ' -f1)
actual=$(docker run --rm --entrypoint sha256sum "$image" /fastdds_profiles.xml /dds_check.sh | cut -d' ' -f1)
[[ $expected == "$actual" ]] || fail "sha256 of /fastdds_profiles.xml, /dds_check.sh differs from ros2/dds/"

if ((failure_count > 0)); then
  exit 1
fi
echo "$image: DDS files, ENV and ENTRYPOINT are as expected"
