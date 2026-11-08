# syntax=docker/dockerfile:1
# Build context is the repository root (ros2/ddt_diff_drive/ddt_diff_drive/ddt_bridge.py links to host/).
# Base digest taken 2026-11-06 (docker buildx imagetools inspect ros:jazzy-ros-base-noble).
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# Fast DDS is configured by /fastdds_profiles.xml (ros2/dds/, copied into every tsukuchalle image):
# SHM with a 16 MiB segment plus UDPv4 on 127.0.0.1 only, no multicast, so best-effort 1.5 MB point
# clouds do not overflow and nothing reaches the LAN. The transports and the discovery range must come
# from the XML and ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT, not from LOCALHOST: LOCALHOST makes
# rmw_fastrtps add a default 512 KiB SHM transport after reading the XML. (Its UDP listens on all
# interfaces; it stays off the LAN only because it has no multicast and its peer is 127.0.0.1.)
# Containers need `ipc: host` and `network_mode: host` (compose does; `docker run` needs `--ipc host`).
# SHM files in /dev/shm belong to the uid that made them (0644, umask does not help), so every ROS
# process on the host must run as the same uid (compose `user:`, TSUKUCHALLE_UID); /dds_check.sh stops a
# mismatch. ROS_LOG_DIR / ROS_HOME are under /tmp because uid 1000 cannot write to / or a missing HOME.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT \
    FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml \
    ROS_LOG_DIR=/tmp/ros_log \
    ROS_HOME=/tmp/ros_home

# The ROS apt repository drops old versions, so the ROS packages are not pinned.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY host host
COPY ros2 ros2
# ros2/velodyne/ holds a package of the Velodyne image; this image builds ddt_diff_drive and tsukuchalle_host_monitor.
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths ros2 --packages-select ddt_diff_drive tsukuchalle_host_monitor

# Last, so editing them does not rebuild the layers above.
COPY --chmod=0644 ros2/dds/fastdds_profiles.xml /fastdds_profiles.xml
COPY --chmod=0755 ros2/dds/dds_check.sh /dds_check.sh
ENTRYPOINT ["/dds_check.sh", "/ws/ros2/docker-entrypoint.sh"]
# Exec the node directly: `ros2 run` would sit between PID 1 and the node and swallow SIGTERM.
CMD ["/ws/install/ddt_diff_drive/lib/ddt_diff_drive/diff_drive_node", "--ros-args", "--params-file", "/ws/install/ddt_diff_drive/share/ddt_diff_drive/config/diff_drive.yaml"]
