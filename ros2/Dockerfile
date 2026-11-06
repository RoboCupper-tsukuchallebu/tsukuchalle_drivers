# syntax=docker/dockerfile:1
# Build context is the repository root (ros2/ddt_diff_drive/ddt_diff_drive/ddt_bridge.py links to host/).
FROM ros:jazzy-ros-base

ENV ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY host host
COPY ros2 ros2
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths ros2

ENTRYPOINT ["/ws/ros2/docker-entrypoint.sh"]
# Exec the node directly: `ros2 run` would sit between PID 1 and the node and swallow SIGTERM.
CMD ["/ws/install/ddt_diff_drive/lib/ddt_diff_drive/diff_drive_node", "--ros-args", "--params-file", "/ws/install/ddt_diff_drive/share/ddt_diff_drive/config/diff_drive.yaml"]
