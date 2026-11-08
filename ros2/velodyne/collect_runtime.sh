#!/usr/bin/env bash
# Copies the two Velodyne executables and every shared library they need into <output_root>,
# keeping their paths, so that `COPY --from=builder <output_root> /` onto ubuntu:noble is enough.
#
# usage: LD_LIBRARY_PATH=<same value as the runtime ENV> collect_runtime.sh <output_root>
# needs: /noble-dpkg-info = /var/lib/dpkg/info of the ubuntu:noble image used for the runtime stage.
#
# - A library that a package of ubuntu:noble already installs (libc6, libstdc++6, libgcc-s1, ...)
#   is not copied: a partly replaced glibc breaks libresolv / libnss_* / GLIBC_PRIVATE.
# - Only the directory part of a path goes through realpath (/lib -> /usr/lib); the file name is
#   kept so SONAME symlinks survive. A symlink is copied together with everything it points to.
# - Libraries that ldd cannot see but that are dlopen()ed at run time are added (see add_dlopen_libraries).
set -euo pipefail

readonly output_root=${1:?usage: collect_runtime.sh <output_root>}
readonly noble_package_lists=/noble-dpkg-info
readonly ros_library_directory=/opt/ros/jazzy/lib
readonly seed_executables=(
  /opt/ros/jazzy/lib/velodyne_driver/velodyne_driver_node
  /ws/install/tsukuchalle_velodyne/lib/tsukuchalle_velodyne/velodyne_pointcloud_node
)
readonly config_file=/ws/install/tsukuchalle_velodyne/share/tsukuchalle_velodyne/config/32db.yaml

work_directory=$(mktemp -d)
trap 'rm -rf "$work_directory"' EXIT
readonly noble_paths=$work_directory/noble_paths
cat "$noble_package_lists"/*.list | sed -E 's#^/(lib|lib64|bin|sbin)/#/usr/\1/#' | sort -u > "$noble_paths"

declare -A queued=()
queue=()

normalize_path() {
  printf '%s/%s\n' "$(realpath "$(dirname "$1")")" "$(basename "$1")"
}

is_provided_by_noble() {
  grep -Fxq -- "$1" "$noble_paths"
}

# Copies <path> and, when it is a symlink, everything it points to. Queues new files for ldd.
copy_with_symlink_chain() {
  local path=$1
  while ! is_provided_by_noble "$path"; do
    if [[ -z ${queued[$path]:-} ]]; then
      queued[$path]=1
      queue+=("$path")
      cp -a --parents "$path" "$output_root"
    fi
    [[ -L $path ]] || break
    path=$(normalize_path "$(dirname "$path")/$(readlink "$path")")
  done
}

# Prints the resolved path of every dependency ldd finds; fails when one is missing.
list_dependencies() {
  local missing
  missing=$(ldd "$1" | awk '$2 == "=>" && $3 == "not" {print $1}')
  if [[ -n $missing ]]; then
    echo "collect_runtime: $1 needs, but cannot find: $missing" >&2
    return 1
  fi
  ldd "$1" | awk '$2 == "=>" && $3 ~ /^\// {print $3} $1 ~ /^\// && $2 ~ /^\(0x/ {print $1}'
}

# rmw_implementation dlopen()s librmw_fastrtps_cpp.so by name; rosidl_typesupport_{c,cpp} dlopen()
# lib<pkg>__rosidl_typesupport_{fastrtps,introspection}_{c,cpp}.so next to the library ldd shows.
add_dlopen_libraries() {
  local library_name=$1 prefix candidate
  case $library_name in
    *__rosidl_typesupport_c.so* | *__rosidl_typesupport_cpp.so*)
      prefix=${library_name%%__rosidl_typesupport_*}__rosidl_typesupport_
      for candidate in fastrtps_cpp fastrtps_c introspection_cpp introspection_c; do
        if [[ -e $ros_library_directory/$prefix$candidate.so ]]; then
          copy_with_symlink_chain "$ros_library_directory/$prefix$candidate.so"
        fi
      done
      ;;
  esac
}

mkdir -p "$output_root"
for executable in "${seed_executables[@]}"; do
  copy_with_symlink_chain "$(normalize_path "$executable")"
done
copy_with_symlink_chain "$ros_library_directory/librmw_fastrtps_cpp.so"

# Fixed point: every file added by ldd or by the dlopen rule is examined again.
while ((${#queue[@]} > 0)); do
  current=${queue[0]}
  queue=("${queue[@]:1}")
  add_dlopen_libraries "$(basename "$current")"
  dependencies=$(list_dependencies "$current")   # an assignment, so a failure stops the script
  while read -r dependency; do
    [[ -z $dependency ]] || copy_with_symlink_chain "$(normalize_path "$dependency")"
  done <<< "$dependencies"
done

mkdir -p "$output_root$(dirname "$config_file")"
cp -a "$config_file" "$output_root$config_file"

# /lib, /lib64, /bin and /sbin are symlinks to /usr/... on ubuntu:noble; a real directory of the same
# name in the copied tree would clash with them.
for symlinked_directory in lib lib64 bin sbin; do
  if [[ -e $output_root/$symlinked_directory ]]; then
    echo "collect_runtime: $output_root/$symlinked_directory must not exist (realpath missed a path)" >&2
    exit 1
  fi
done

# Every dependency of every collected file is either collected or provided by ubuntu:noble.
while IFS= read -r collected_file; do
  dependencies=$(list_dependencies "$collected_file")
  while read -r dependency; do
    [[ -n $dependency ]] || continue
    normalized=$(normalize_path "$dependency")
    if ! is_provided_by_noble "$normalized" && [[ ! -e $output_root$normalized ]]; then
      echo "collect_runtime: $normalized (needed by $collected_file) was not collected" >&2
      exit 1
    fi
  done <<< "$dependencies"
done < <(printf '%s\n' "${!queued[@]}" | sort)

file_count=$(find "$output_root" \( -type f -o -type l \) | wc -l)
size_kib=$(du -sk "$output_root" | cut -f1)
echo "collect_runtime: ${file_count} files (symlinks included), ${size_kib} KiB in ${output_root}"
