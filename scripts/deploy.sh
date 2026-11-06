#!/bin/bash
# Run on the build machine inside the clone, after checking out the commit to ship.
# Usage: scripts/deploy.sh <robot-ssh-target>
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <robot-ssh-target>" >&2
    exit 2
fi
robotTarget="$1"
repositoryRoot="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
imageName="ddt-can-bridge-ros2"
imageTag="$(git -C "${repositoryRoot}" rev-parse --short HEAD)"

docker build -f "${repositoryRoot}/ros2/Dockerfile" \
    -t "${imageName}:${imageTag}" -t "${imageName}:latest" "${repositoryRoot}"
docker save "${imageName}:${imageTag}" "${imageName}:latest" | gzip | ssh "${robotTarget}" "gunzip | docker load"
echo "deployed ${imageName}:${imageTag} (also latest) to ${robotTarget}"
