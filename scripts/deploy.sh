#!/bin/bash
# Run on the build machine inside the clone. Builds the committed tree and ships it to the robot PC.
# Usage: scripts/deploy.sh <robot-ssh-destination> <ddt|velodyne|all>
# The image choice is required: the velodyne image carries PCL, so ddt-only changes should not ship it.
# The tools image is always shipped for diagnostics and SHM cleanup. Images are shipped one by one;
# if a later build fails, earlier images are already latest, but files are not sent yet.
set -euo pipefail

if [ "$#" -ne 2 ] || [[ "$1" == -* ]]; then
    printf "usage: %s <robot-ssh-destination> <ddt|velodyne|all>\nThe tools image and cleanup script are always shipped.\n" "$0" >&2
    exit 2
fi
robotTarget="$1"
case "$2" in
    ddt)      imageSelections=(ddt) ;;
    velodyne) imageSelections=(velodyne) ;;
    all)      imageSelections=(ddt velodyne) ;;
    *)
        printf "usage: %s <robot-ssh-destination> <ddt|velodyne|all>\nThe tools image and cleanup script are always shipped.\n" "$0" >&2
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

imageSelections+=(tools)
deployedImages=()
for imageSelection in "${imageSelections[@]}"; do
    case "${imageSelection}" in
        ddt)      imageName="ddt-can-bridge-ros2";  dockerfilePath="ros2/Dockerfile" ;;
        velodyne) imageName="tsukuchalle-velodyne"; dockerfilePath="ros2/velodyne/Dockerfile" ;;
        tools)    imageName="tsukuchalle-ros-tools"; dockerfilePath="ros2/tools/Dockerfile" ;;
    esac
    docker build -f "${dockerfilePath}" -t "${imageName}:${imageTag}" .
    docker save "${imageName}:${imageTag}" | gzip | ssh "${robotTarget}" "gunzip | docker load"
    # Image names and SHA are set locally and intentionally expanded before SSH.
    # shellcheck disable=SC2029
    ssh "${robotTarget}" "docker tag ${imageName}:${imageTag} ${imageName}:latest"
    deployedImages+=("${imageName}:${imageTag}")
done

ssh "${robotTarget}" "mkdir -p ~/ddt-can-bridge"
scp -r ros2/compose.yaml ros2/config scripts/udev ros2/tools/dds_shm_clean.sh "${robotTarget}:~/ddt-can-bridge/"
ssh "${robotTarget}" "chmod +x ~/ddt-can-bridge/dds_shm_clean.sh"

cat <<MESSAGE
deployed ${deployedImages[*]} (also latest) and compose files, udev rules, cleanup script to ${robotTarget}:~/ddt-can-bridge/
On the robot PC:
  sudo bash ~/ddt-can-bridge/udev/install.sh   # udev rules (first time, or after changing them)
  cd ~/ddt-can-bridge
.env (see README): DDT_SERIAL_DEVICE, and when this PC differs from X13 (uid/gid 1000, dialout 20, input 995):
  TSUKUCHALLE_UID TSUKUCHALLE_GID TSUKUCHALLE_DIALOUT_GID TSUKUCHALLE_INPUT_GID  (id -u, id -g, getent group dialout input)
  Every ROS container on the PC must run as one uid; TSUKUCHALLE_UID=0 TSUKUCHALLE_GID=0 until the other repositories are updated.
MESSAGE
if [[ " ${imageSelections[*]} " == *" ddt "* ]]; then
    cat <<'MESSAGE'
Drive (wheels lifted):
  export DDT_SERIAL_DEVICE=/dev/serial/by-id/usb-...
  docker compose up -d drive joy teleop mux
  docker compose exec drive /ws/ros2/docker-entrypoint.sh ros2 service call /diff_drive_node/enable std_srvs/srv/SetBool "{data: true}"
MESSAGE
fi
if [[ " ${imageSelections[*]} " == *" velodyne "* ]]; then
    cat <<'MESSAGE'
Velodyne (DDT_SERIAL_DEVICE is still needed in .env or the environment):
  docker compose --profile velodyne up -d velodyne-driver velodyne-transform
MESSAGE
fi
