# Ubuntu 18.04LTS, Gazebo 11, PX4, ROS2 Simulation container.

# Start with base ROS2 image.
FROM ubuntu1804ros2eloquentdesktop:v1

# Allow apt install to work properly.
ENV DEBIAN_FRONTEND noninteractive
# Use computer locale to allow for improved scripting.
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Copy scripts directory into container.
COPY ./scripts /scripts

# Create user with UID and GID matching those on my PC.
# This allows the user to delete build products created in the source tree.
RUN groupadd -g 1000 build && \
    useradd -u 1000 -g 1000 -m -s /bin/bash build && \
    echo "build ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/build && \
    chmod 0440 /etc/sudoers.d/build
ENV HOME /home/build

# Installation is done as root.
# Set git directory to user build's home directory.
ENV USER_DIR /home/build
# Run one script at a time to allow the image to be built in layers.
RUN cd /scripts/install && bash -x ./install_ros2_gazebo11.bash
RUN cd /scripts/install && bash -x ./install_fast_rtps.bash
RUN cd /scripts/install && bash -x ./install_px4_repos.bash
RUN cd /scripts/install && bash -x ./install_drone_packages.bash

# Change ownership of code repos and workspace to build:build
RUN chown -R build:build /home/build/

# Change to user and build everything.
USER build
RUN cd /scripts/build && bash -x ./build_all.bash
