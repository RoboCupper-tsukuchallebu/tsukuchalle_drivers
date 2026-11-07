#!/usr/bin/env bash
# gtest of tsukuchalle_velodyne: the Release build, then the AddressSanitizer + UBSan build
# (the benchmark limit is only asserted in the first one).
set -euo pipefail
# setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES ...), so nounset is off while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /ws

for variant in test asan; do
  echo "=== gtest: build-${variant} ==="
  if [[ $variant == asan ]]; then
    export ASAN_OPTIONS=detect_leaks=0
    export UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1
  fi
  colcon test --packages-select tsukuchalle_velodyne \
    --build-base "build-${variant}" --install-base "install-${variant}" \
    --event-handlers console_direct+ --return-code-on-test-failure
  colcon test-result --test-result-base "build-${variant}" --verbose
done
