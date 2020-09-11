# Copied from
# https://github.com/osrf/docker_images/blob/0b33e61b5bbed5b93b9fba2d5bae5db604ff9b58/ros/eloquent/ubuntu/bionic/ros-core/Dockerfile
# and changed installation of ROS2 packages.

FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install extra packages.
# sudo is needed by install scripts.
# apt-utils make for neater apt installs.
# gpg et al are needed for adding keys for ROS packages etc.
# dirmngr is used by GPG.
# wget is used to fetch apt keys.
RUN apt-get update && apt-get -q -y --no-install-recommends install \
    sudo \
    apt-utils \
    gnupg2 dirmngr \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Installation is done as root.
# Copy scripts directory into container.
COPY ./scripts /scripts

# ROS2 is massive!
RUN cd /scripts/install && bash -x ./install_ros2.bash
