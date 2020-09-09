# Ubuntu 18.04LTS, Gazebo 11, PX4, ROS2 Simulation container.

# Start with base ROS2 image.
FROM ubuntu1804ros2eloquentdesktop:v1

# Allow apt install to work properly.
ENV DEBIAN_FRONTEND noninteractive
# Use computer locale to allow for improved scripting.
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Copy scripts directory into container.
COPY ./scripts /scripts

# Installation is done as root.
# Run one script at a time to allow the image to be built in layers.
RUN cd /scripts/install && bash -x ./install_ros2_gazebo11.bash
RUN cd /scripts/install && bash -x ./install_fast_rtps.bash
RUN cd /scripts/install && bash -x ./install_px4_repos.bash
RUN cd /scripts/install && bash -x ./install_drone_packages.bash

# Build is done as user.
# Set user to Ubuntu first user.  This allows the user to delete build products
# created in the source tree.
RUN groupadd --gid 1000 build
RUN useradd --uid 1000 -m -s /bin/bash build
USER build
RUN whoami && echo "HOME = $HOME"

# RUN cd /scripts/build && ./build_all.bash
