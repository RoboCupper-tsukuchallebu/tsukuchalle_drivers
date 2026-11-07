#!/bin/bash
# Run on the build machine inside the clone. Builds the committed tree and ships it to the robot PC.
# Usage: scripts/deploy.sh <robot-ssh-destination> <ddt|velodyne|all>
# The image choice is required: the velodyne image carries PCL, so ddt-only changes should not ship it.
# With "all", images are shipped one by one: if the second build fails, the first image is already
# loaded and tagged latest on the robot, but compose.yaml and udev/ are not sent yet.
set -euo pipefail

if [ "$#" -ne 2 ] || [[ "$1" == -* ]]; then
    echo "usage: $0 <robot-ssh-destination> <ddt|velodyne|all>" >&2
    exit 2
fi
robotTarget="$1"
case "$2" in
    ddt)      imageSelections=(ddt) ;;
    velodyne) imageSelections=(velodyne) ;;
    all)      imageSelections=(ddt velodyne) ;;
    *)
        echo "usage: $0 <robot-ssh-destination> <ddt|velodyne|all>" >&2
        exit 2
        ;;
esac
repositoryRoot="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repositoryRoot}"

if [ -n "$(git status --porcelain)" ]; then
    echo "uncommitted changes: commit before running (the image tag is the commit SHA)" >&2
    exit 1
fi
imageTag="$(git rev-parse --short HEAD)"

buildArchitecture="$(uname -m)"
robotArchitecture="$(ssh "${robotTarget}" uname -m)"
if [ "${buildArchitecture}" != "${robotArchitecture}" ]; then
    echo "architecture mismatch: build=${buildArchitecture} robot=${robotArchitecture}" >&2
    exit 1
fi

deployedImages=()
for imageSelection in "${imageSelections[@]}"; do
    case "${imageSelection}" in
        ddt)      imageName="ddt-can-bridge-ros2";  dockerfilePath="ros2/Dockerfile" ;;
        velodyne) imageName="tsukuchalle-velodyne"; dockerfilePath="ros2/velodyne/Dockerfile" ;;
    esac
    docker build -f "${dockerfilePath}" -t "${imageName}:${imageTag}" .
    docker save "${imageName}:${imageTag}" | gzip | ssh "${robotTarget}" "gunzip | docker load"
    ssh "${robotTarget}" "docker tag ${imageName}:${imageTag} ${imageName}:latest"
    deployedImages+=("${imageName}:${imageTag}")
done

ssh "${robotTarget}" "mkdir -p ~/ddt-can-bridge"
scp -r ros2/compose.yaml ros2/config scripts/udev "${robotTarget}:~/ddt-can-bridge/"

cat <<MESSAGE
deployed ${deployedImages[*]} (also latest) and compose files, udev rules to ${robotTarget}:~/ddt-can-bridge/
On the robot PC:
  sudo bash ~/ddt-can-bridge/udev/install.sh   # udev rules (first time, or after changing them)
  cd ~/ddt-can-bridge
MESSAGE
if [[ " ${imageSelections[*]} " == *" ddt "* ]]; then
    cat <<'MESSAGE'
Drive (wheels lifted):
  export DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...
  docker compose up -d drive joy teleop
  docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
MESSAGE
fi
if [[ " ${imageSelections[*]} " == *" velodyne "* ]]; then
    cat <<'MESSAGE'
Velodyne (DDT_SERIAL_DEVICE is still needed in .env or the environment):
  docker compose --profile velodyne up -d velodyne-driver velodyne-transform
MESSAGE
fi
