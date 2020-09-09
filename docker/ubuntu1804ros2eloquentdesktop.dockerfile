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

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install sudo as needed by install scripts.
# apt-utils make for neater apt installs.
# gpg et al are needed for adding keys for ROS packages etc.
# dirmngr is used by GPG.
# curl is used to fetch time zone info.
# wget is used to fetch apt keys.
RUN apt-get update && apt-get -q -y --no-install-recommends install \
    sudo \
    apt-utils \
    gnupg2 dirmngr \
    curl \
    wget

# Installation is done as root.
# Copy scripts directory into container.
COPY ./scripts /scripts

# ROS2 is massive!
RUN cd /scripts/install && bash -x ./install_ros2.bash
