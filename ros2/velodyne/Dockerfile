# syntax=docker/dockerfile:1
# Separate from ros2/Dockerfile: ros-jazzy-velodyne depends on libpcl-all-dev (PCL and VTK),
# and scripts/deploy.sh ships whole images, so one shared image would send PCL on every deploy.
# Build context is the repository root, same as ros2/Dockerfile (nothing is COPYed).
# Base digest is the same as ros2/Dockerfile (taken 2026-11-06).
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# See ros2/Dockerfile for why LOCALHOST discovery needs `ipc: host`.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# The ROS apt repository drops old versions, so the ROS package is not pinned.
# ENTRYPOINT stays the base image's /ros_entrypoint.sh.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-velodyne \
    && rm -rf /var/lib/apt/lists/*
