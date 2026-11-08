#!/bin/sh
# Tests dds_check.sh. Usage:
#   ros2/dds/test_dds_check.sh            the script in this directory, run on this machine
#   ros2/dds/test_dds_check.sh <image>    the copy inside a built image (docker run, as uid 1000 and 0)
# In image mode the image's own ENTRYPOINT runs dds_check.sh first, so the image's `true`/`printenv`
# below prove that the check sits in front of every command.
set -u

script_directory=$(cd "$(dirname "$0")" && pwd)
image=${1:-}
failure_count=0

cases_directory=$(mktemp -d)
trap 'rm -rf "$cases_directory"' EXIT
cp "$script_directory/fastdds_profiles.xml" "$cases_directory/good.xml"
printf '<profiles><participant profile_name="x"/></profiles>\n' > "$cases_directory/no_default.xml"
printf 'this is not xml\n' > "$cases_directory/garbage.xml"
chmod -R a+rX "$cases_directory"

if [ -n "$image" ]; then
  profiles_in_case=/cases
else
  profiles_in_case=$cases_directory
fi
good_xml="$profiles_in_case/good.xml"

# run_case <user> <variable assignments...> -- <command...>; prints the command's output, returns its status.
run_case() {
  case_user=$1
  shift
  assignments=
  while [ "$1" != -- ]; do
    assignments="$assignments $1"
    shift
  done
  shift
  if [ -n "$image" ]; then
    environment_options=
    for assignment in $assignments; do
      environment_options="$environment_options -e $assignment"
    done
    # shellcheck disable=SC2086
    docker run --rm --user "$case_user" -v "$cases_directory:/cases:ro" $environment_options "$image" "$@"
  else
    # shellcheck disable=SC2086
    env $assignments "$script_directory/dds_check.sh" "$@"
  fi
}

expect_pass() {
  description=$1
  shift
  if output=$(run_case "$@" 2>&1); then
    echo "ok   $description"
  else
    echo "FAIL $description (expected exit 0): $output"
    failure_count=$((failure_count + 1))
  fi
}

# expect_stop <description> <text the message must contain> <run_case arguments...>
expect_stop() {
  description=$1
  message=$2
  shift 2
  output=$(run_case "$@" 2>&1)
  status=$?
  if [ "$status" -eq 1 ] && printf '%s' "$output" | grep -q "$message"; then
    echo "ok   $description"
  else
    echo "FAIL $description (expected exit 1 with '$message', got $status): $output"
    failure_count=$((failure_count + 1))
  fi
}

if [ -n "$image" ]; then
  current_uid=1000
  current_user=1000:1000
else
  current_uid=$(id -u)
  current_user=$current_uid
fi
expect_pass "valid xml, matching uid, command is executed" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
  TSUKUCHALLE_UID="$current_uid" -- true
expect_pass "TSUKUCHALLE_UID unset skips the uid check" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -- true

expect_stop "uid differs from TSUKUCHALLE_UID" "uid" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
  TSUKUCHALLE_UID=4242 -- true
expect_stop "xml file is missing" "FASTRTPS_DEFAULT_PROFILES_FILE" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$profiles_in_case/missing.xml" \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -- true
expect_stop "xml has no default profile" "is_default_profile" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$profiles_in_case/no_default.xml" \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -- true
expect_stop "xml is garbage" "is_default_profile" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$profiles_in_case/garbage.xml" \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -- true
expect_stop "discovery range is LOCALHOST" "ROS_AUTOMATIC_DISCOVERY_RANGE" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -- true
expect_stop "discovery range is SUBNET" "ROS_AUTOMATIC_DISCOVERY_RANGE" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -- true

# lan mode: xml and discovery range are not checked and are removed for the command; the uid check stays.
expect_pass "lan mode: missing xml and SUBNET pass" \
  "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$profiles_in_case/missing.xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  TSUKUCHALLE_DDS_MODE=lan -- true
# After dds_check.sh the command may run behind a ROS entrypoint, and /opt/ros/jazzy/setup.bash exports an
# unset ROS_AUTOMATIC_DISCOVERY_RANGE as SUBNET (the default), so "removed" means unset or SUBNET.
# shellcheck disable=SC2016 # the single quotes are deliberate: the command's own sh expands them
lan_output=$(run_case "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
  TSUKUCHALLE_DDS_MODE=lan -- sh -c 'echo "profile=${FASTRTPS_DEFAULT_PROFILES_FILE-unset} range=${ROS_AUTOMATIC_DISCOVERY_RANGE-unset}"' 2>/dev/null)
case "$lan_output" in
  "profile=unset range=unset" | "profile=unset range=SUBNET") echo "ok   lan mode removes the xml and SYSTEM_DEFAULT for the command" ;;
  *)
    echo "FAIL lan mode leaves the settings for the command: $lan_output"
    failure_count=$((failure_count + 1))
    ;;
esac
expect_stop "lan mode still checks the uid" "uid" \
  "$current_user" TSUKUCHALLE_DDS_MODE=lan TSUKUCHALLE_UID=4242 -- true

if [ -z "$image" ]; then
  output=$(run_case "$current_user" FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -- 2>&1)
  if [ "$?" -eq 2 ]; then
    echo "ok   no command is a usage error"
  else
    echo "FAIL no command should exit 2: $output"
    failure_count=$((failure_count + 1))
  fi
fi

if [ -n "$image" ]; then
  expect_pass "uid 0 with TSUKUCHALLE_UID=0 (temporary root operation)" \
    0:0 FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT TSUKUCHALLE_UID=0 -- true
  expect_stop "uid 0 with TSUKUCHALLE_UID=1000" "uid" \
    0:0 FASTRTPS_DEFAULT_PROFILES_FILE="$good_xml" ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT TSUKUCHALLE_UID=1000 -- true
  # The image's own ENV must satisfy the check without any -e.
  if docker run --rm --user 1000:1000 -e TSUKUCHALLE_UID=1000 "$image" true; then
    echo "ok   the image's default ENV passes the check"
  else
    echo "FAIL the image's default ENV does not pass the check"
    failure_count=$((failure_count + 1))
  fi
fi

if [ "$failure_count" -gt 0 ]; then
  echo "$failure_count case(s) failed"
  exit 1
fi
echo "dds_check tests passed"
