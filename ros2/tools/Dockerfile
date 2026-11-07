# syntax=docker/dockerfile:1
# Debug / CI tools for the Velodyne pipeline: ros2 topic hz / echo, ros2 bag record /velodyne_packets,
# and the Python test in ros2/velodyne/. The minimal tsukuchalle-velodyne image has none of them.
# Build context is the repository root (nothing is COPYed). Base digest is the same as ros2/Dockerfile.
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# See ros2/Dockerfile for why LOCALHOST discovery needs `--ipc host`.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# The ROS apt repository drops old versions, so the ROS package is not pinned.
# ENTRYPOINT stays the base image's /ros_entrypoint.sh.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-velodyne-msgs \
    && rm -rf /var/lib/apt/lists/*
