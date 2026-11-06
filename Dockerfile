# Firmware and test toolchain. Sources are not copied in; the Makefile bind-mounts them at /work.
# Base digest fetched with `docker buildx imagetools inspect ubuntu:24.04` on 2026-11-06.
FROM ubuntu:24.04@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852

# Host-side tools are not pinned: a base image digest does not freeze apt versions,
# and they do not affect the firmware binary (the three pins below do).
# build-essential is required for pioasm, picotool and the host-built tests.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        cmake ninja-build git build-essential python3 curl xz-utils ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Arm GNU Toolchain 13.3.Rel1. SHA256 values come from the .sha256asc files on Arm's download page.
ARG TARGETARCH
ARG ARM_TOOLCHAIN_SHA256_AMD64=95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4
ARG ARM_TOOLCHAIN_SHA256_ARM64=c8824bffd057afce2259f7618254e840715f33523a3d4e4294f471208f976764
RUN set -eu; \
    case "${TARGETARCH}" in \
        amd64) toolchain_host=x86_64;  toolchain_sha256="${ARM_TOOLCHAIN_SHA256_AMD64}" ;; \
        arm64) toolchain_host=aarch64; toolchain_sha256="${ARM_TOOLCHAIN_SHA256_ARM64}" ;; \
        *) echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    toolchain_archive="arm-gnu-toolchain-13.3.rel1-${toolchain_host}-arm-none-eabi.tar.xz"; \
    curl -fsSL -o "/tmp/${toolchain_archive}" \
        "https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/${toolchain_archive}"; \
    echo "${toolchain_sha256}  /tmp/${toolchain_archive}" | sha256sum -c -; \
    mkdir /opt/arm-gnu-toolchain; \
    tar -xJf "/tmp/${toolchain_archive}" -C /opt/arm-gnu-toolchain --strip-components=1; \
    rm "/tmp/${toolchain_archive}"
ENV PATH="/opt/arm-gnu-toolchain/bin:${PATH}"

# Pico SDK 2.3.1 with only the tinyusb submodule (same layout as the developer machines).
RUN git clone --depth 1 --branch 2.3.1 https://github.com/raspberrypi/pico-sdk /opt/pico-sdk \
    && git -C /opt/pico-sdk submodule update --init --depth 1 lib/tinyusb
ENV PICO_SDK_PATH=/opt/pico-sdk

# picotool 2.3.1 (same version as the SDK). Installed under /usr/local so find_package(picotool) finds it
# and the SDK never fetches it at build time.
RUN git clone --depth 1 --branch 2.3.1 https://github.com/raspberrypi/picotool /tmp/picotool \
    && cmake -S /tmp/picotool -B /tmp/picotool/build -G Ninja \
        -DPICO_SDK_PATH=/opt/pico-sdk -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/picotool/build \
    && cmake --install /tmp/picotool/build \
    && rm -rf /tmp/picotool

WORKDIR /work
