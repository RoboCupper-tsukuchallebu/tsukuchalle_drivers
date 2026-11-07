# syntax=docker/dockerfile:1
# Build context is the repository root, same as ros2/Dockerfile.
#
# Stages: base -> builder (colcon build + collect_runtime.sh) -> test (CI only), and runtime (last,
# so a plain `docker build` makes it). `runtime` is ubuntu:noble plus the files collect_runtime.sh
# gathered from `builder`; it has no apt-installed ROS, no compiler and no PCL, so scripts/deploy.sh
# sends tens of MiB instead of ~1 GiB. There is no ENTRYPOINT and setup.bash is not sourced: the
# compose `command` runs the executables directly. For ros2 CLI / rosbag use ros2/tools/Dockerfile.

# Digests taken 2026-11-07. The ubuntu digest is the index whose amd64 root layer is the bottom layer of
# the ros base image below (compare with `docker image inspect --format '{{index .RootFS.Layers 0}}'`),
# so the libc6 / libstdc++6 of builder and runtime are the same package versions (CI checks it).
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
ARG UBUNTU_IMAGE=ubuntu:noble@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852
# Shared by collect_runtime.sh (ldd in `builder`) and the ENV of `runtime`, so a directory that ldd
# needs cannot be missing from the runtime search path.
ARG RUNTIME_LIBRARY_PATH=/opt/ros/jazzy/lib:/ws/install/tsukuchalle_velodyne/lib

FROM ${UBUNTU_IMAGE} AS ubuntu-noble

FROM ${ROS_BASE_IMAGE} AS base
# The ROS apt repository drops old versions, so the ROS packages are not pinned.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libyaml-cpp-dev \
        ros-jazzy-velodyne-driver \
        ros-jazzy-velodyne-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws

FROM base AS builder
ARG RUNTIME_LIBRARY_PATH
COPY ros2/velodyne/tsukuchalle_velodyne src/tsukuchalle_velodyne
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select tsukuchalle_velodyne \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY --from=ubuntu-noble /var/lib/dpkg/info /noble-dpkg-info
COPY ros2/velodyne/collect_runtime.sh /usr/local/bin/collect_runtime.sh
RUN LD_LIBRARY_PATH=${RUNTIME_LIBRARY_PATH} collect_runtime.sh /runtime-root

# gtest (Release, then AddressSanitizer + UBSan). CI runs it with `docker run`, not in a RUN, so
# a failure is not cut off by BuildKit and a cached layer cannot skip the tests.
FROM builder AS test
COPY ros2/velodyne/run_gtest.sh /usr/local/bin/run_gtest.sh
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select tsukuchalle_velodyne \
        --build-base build-test --install-base install-test \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    && colcon build --packages-select tsukuchalle_velodyne \
        --build-base build-asan --install-base install-asan \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
            "-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer"
CMD ["run_gtest.sh"]

FROM ${UBUNTU_IMAGE} AS runtime
ARG RUNTIME_LIBRARY_PATH
COPY --from=builder /runtime-root /
# LD_LIBRARY_PATH must stay. rmw_implementation dlopen()s librmw_fastrtps_cpp.so and
# rosidl_typesupport_cpp finds lib<pkg>__rosidl_typesupport_*.so with rcpputils::find_library_path,
# which only looks in the directories of LD_LIBRARY_PATH and never in ld.so.cache. Replacing it with
# ldconfig makes the nodes fail at startup. Every library directory collect_runtime.sh used is in it
# (CI checks that).
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST needs `ipc: host` (see ros2/Dockerfile).
ENV LANG=C.UTF-8 \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    LD_LIBRARY_PATH=${RUNTIME_LIBRARY_PATH} \
    ROS_LOG_DIR=/tmp/ros_log \
    ROS_DOMAIN_ID=42 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4
