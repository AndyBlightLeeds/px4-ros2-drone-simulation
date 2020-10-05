# Copied from
# https://github.com/osrf/docker_images/blob/30496b03a10f37d3ee54d5df672a0c27e0ab3952/ros/foxy/ubuntu/focal/desktop/Dockerfile

# Added NVidia and user build sections.

FROM ros:foxy-ros-base-focal

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# NVidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Create user with UID and GID matching those on my PC.
# This allows the user to delete build products created in the source tree.
# Also add user build to sudoers list.
RUN groupadd -g 1000 build && \
    useradd -u 1000 -g 1000 -m -s /bin/bash build && \
    echo "build ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/build && \
    chmod 0440 /etc/sudoers.d/build
ENV HOME /home/build

# Change ownership of build directory to build:build.
RUN chown -R build:build /home/build/

# Change to user build.
USER build
