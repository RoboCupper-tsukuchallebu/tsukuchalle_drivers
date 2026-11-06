#!/bin/bash
# Run on the build machine inside the clone. Builds the committed tree and ships it to the robot PC.
# Usage: scripts/deploy.sh <robot-ssh-destination>
set -euo pipefail

if [ "$#" -ne 1 ] || [[ "$1" == -* ]]; then
    echo "usage: $0 <robot-ssh-destination>" >&2
    exit 2
fi
robotTarget="$1"
repositoryRoot="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repositoryRoot}"

if [ -n "$(git status --porcelain)" ]; then
    echo "uncommitted changes: commit before running (the image tag is the commit SHA)" >&2
    exit 1
fi
imageName="ddt-can-bridge-ros2"
imageTag="$(git rev-parse --short HEAD)"

buildArchitecture="$(uname -m)"
robotArchitecture="$(ssh "${robotTarget}" uname -m)"
if [ "${buildArchitecture}" != "${robotArchitecture}" ]; then
    echo "architecture mismatch: build=${buildArchitecture} robot=${robotArchitecture}" >&2
    exit 1
fi

docker build -f ros2/Dockerfile -t "${imageName}:${imageTag}" .
docker save "${imageName}:${imageTag}" | gzip | ssh "${robotTarget}" "gunzip | docker load"
ssh "${robotTarget}" "docker tag ${imageName}:${imageTag} ${imageName}:latest"

ssh "${robotTarget}" "mkdir -p ~/ddt-can-bridge"
scp -r ros2/compose.yaml ros2/config "${robotTarget}:~/ddt-can-bridge/"

cat <<MESSAGE
deployed ${imageName}:${imageTag} (also latest) and compose files to ${robotTarget}:~/ddt-can-bridge/
On the robot PC (wheels lifted):
  cd ~/ddt-can-bridge
  export DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...
  docker compose up -d drive joy teleop
  docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
MESSAGE
