#!/bin/sh
# Stop every ROS process before running; do not start one during cleanup.
set -eu

case ${1-} in
  '') dry_run=false ;;
  --dry-run) dry_run=true ;;
  *) echo "Usage: $0 [--dry-run]" >&2; exit 2 ;;
esac
if [ "$#" -gt 1 ]; then
  echo "Usage: $0 [--dry-run]" >&2
  exit 2
fi

# Any running container blocks cleanup. Matching image names missed ROS images from other
# repositories (for example lightnav started with docker run), and cleanup is rare enough
# that stopping everything first costs little. Docker errors must prevent cleanup.
active=$(docker ps --format '{{.Names}}\t{{.Image}}\t{{.Label "com.docker.compose.project"}}') || {
  echo 'Could not list containers; cleanup aborted.' >&2
  exit 1
}
if [ -n "$active" ]; then
  printf 'Running containers prevent cleanup (name, image, Compose project):\n%s\n' "$active" >&2
  exit 1
fi
uid=${TSUKUCHALLE_UID:-1000}
gid=${TSUKUCHALLE_GID:-1000}
if [ "$dry_run" = true ]; then
  printf 'Would run fastdds shm clean in tsukuchalle-ros-tools with host IPC and user %s:%s.\n' "$uid" "$gid"
  exit 0
fi
exec docker run --rm --ipc host --pid host --user "$uid:$gid" -e "TSUKUCHALLE_UID=$uid" tsukuchalle-ros-tools fastdds shm clean
