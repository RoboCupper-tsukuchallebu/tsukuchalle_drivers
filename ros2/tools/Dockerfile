# syntax=docker/dockerfile:1
# Debug / CI tools for the Velodyne pipeline: ros2 topic hz / echo, ros2 bag record /velodyne_packets,
# and the Python test in ros2/velodyne/. The minimal tsukuchalle-velodyne image has none of them.
# Build context is the repository root (only ros2/dds/ is COPYed). Base digest is the same as ros2/Dockerfile.
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# Fast DDS profile, uid and log directories: see ros2/Dockerfile. Run with `--user "$(id -u):$(id -g)"
# -e TSUKUCHALLE_UID=...` (README), or /dds_check.sh stops it when the uid differs.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
    FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml \
    ROS_LOG_DIR=/tmp/ros_log \
    ROS_HOME=/tmp/ros_home

# The ROS apt repository drops old versions, so the ROS package is not pinned.
# The base image's /ros_entrypoint.sh stays behind /dds_check.sh.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-velodyne-msgs \
    && rm -rf /var/lib/apt/lists/*
# Last, so editing them does not rebuild the layers above.
COPY --chmod=0644 ros2/dds/fastdds_profiles.xml /fastdds_profiles.xml
COPY --chmod=0755 ros2/dds/dds_check.sh /dds_check.sh
ENTRYPOINT ["/dds_check.sh", "/ros_entrypoint.sh"]
CMD ["bash"]
