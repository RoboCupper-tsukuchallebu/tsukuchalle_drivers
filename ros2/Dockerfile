# syntax=docker/dockerfile:1
# Build context is the repository root (ros2/ddt_diff_drive/ddt_diff_drive/ddt_bridge.py links to host/).
# Base digest taken 2026-11-06 (docker buildx imagetools inspect ros:jazzy-ros-base-noble).
FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617

# LOCALHOST discovery makes Fast DDS pick shared memory for peers on the same host, and
# FASTDDS_BUILTIN_TRANSPORTS has no effect then. Containers therefore need `ipc: host`
# (compose does this; `docker run` needs `--ipc host`). UDPv4 only matters after
# overriding the range, e.g. SUBNET.
ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

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
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths ros2

ENTRYPOINT ["/ws/ros2/docker-entrypoint.sh"]
# Exec the node directly: `ros2 run` would sit between PID 1 and the node and swallow SIGTERM.
CMD ["/ws/install/ddt_diff_drive/lib/ddt_diff_drive/diff_drive_node", "--ros-args", "--params-file", "/ws/install/ddt_diff_drive/share/ddt_diff_drive/config/diff_drive.yaml"]
