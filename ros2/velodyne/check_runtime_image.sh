#!/usr/bin/env bash
# Static checks of the minimal tsukuchalle-velodyne image. Run it inside that image:
#   docker run --rm -i --entrypoint bash tsukuchalle-velodyne:ci -s < ros2/velodyne/check_runtime_image.sh
# (it is not part of the image, so the image has no extra file).
set -uo pipefail

failure_count=0
fail() {
  echo "FAIL: $*"
  failure_count=$((failure_count + 1))
}

# (2) nothing for building, and no PCL.
for path in /usr/bin/g++ /usr/bin/gcc /usr/bin/cmake /usr/bin/colcon /opt/ros/jazzy/include \
  /usr/include/pcl /ws/build /ws/src; do
  [[ ! -e $path ]] || fail "$path exists"
done
if compgen -G '/usr/lib/x86_64-linux-gnu/libpcl_common.so*' > /dev/null; then
  fail "libpcl_common is installed"
fi

# (3) no apt-installed ROS package.
if dpkg -l | grep -q '^ii  ros-jazzy'; then
  fail "an apt ros-jazzy package is installed"
fi

# (4) ldd finds every dependency of every shared library and both executables.
checked_count=0
while IFS= read -r file; do
  checked_count=$((checked_count + 1))
  if ldd "$file" 2>&1 | grep -q 'not found'; then
    fail "ldd: $file has a missing dependency: $(ldd "$file" | grep 'not found' | tr -d '\t' | tr '\n' ' ')"
  fi
done < <(find /opt/ros/jazzy/lib /ws/install \( -type f -o -type l \) \
  \( -name '*.so*' -o -name velodyne_driver_node -o -name velodyne_pointcloud_node \) | sort)
echo "ldd checked $checked_count files"

# (5) libraries that are only dlopen()ed.
for library in librmw_fastrtps_cpp.so libvelodyne_msgs__rosidl_typesupport_fastrtps_cpp.so \
  libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so; do
  [[ -e /opt/ros/jazzy/lib/$library ]] || fail "/opt/ros/jazzy/lib/$library is missing"
done

# The dlopen search only uses LD_LIBRARY_PATH (never ld.so.cache): every directory that holds a
# library under /opt/ros or /ws must be listed.
while IFS= read -r directory; do
  case ":${LD_LIBRARY_PATH:-}:" in
    *":$directory:"*) ;;
    *) fail "LD_LIBRARY_PATH lacks $directory" ;;
  esac
done < <(find /opt/ros/jazzy /ws/install \( -type f -o -type l \) -name '*.so*' -printf '%h\n' | sort -u)
IFS=: read -ra listed_directories <<< "${LD_LIBRARY_PATH:-}"
for directory in "${listed_directories[@]}"; do
  [[ -d $directory ]] || fail "LD_LIBRARY_PATH names $directory, which does not exist"
done

# The executables and the calibration file are where compose.yaml expects them.
for path in /opt/ros/jazzy/lib/velodyne_driver/velodyne_driver_node \
  /ws/install/tsukuchalle_velodyne/lib/tsukuchalle_velodyne/velodyne_pointcloud_node; do
  [[ -x $path ]] || fail "$path is not executable"
done
[[ -r /ws/install/tsukuchalle_velodyne/share/tsukuchalle_velodyne/config/32db.yaml ]] ||
  fail "32db.yaml is missing"

if ((failure_count > 0)); then
  echo "$failure_count check(s) failed"
  exit 1
fi
echo "runtime image checks passed"
