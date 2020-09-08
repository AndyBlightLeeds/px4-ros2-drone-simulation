#!/bin/bash
# Install dependencies and setup the iDRO versions of the PX4 repos.
set -ex

. ../project_vars.bash

# Install packages for Firmware and px4_ros_com
sudo apt update
sudo apt -y --no-install-recommends install \
    python3-colcon-common-extensions \
    ros-${ROS2_DISTRO}-eigen3-cmake-module \
    ros-${ROS2_DISTRO}-mavlink \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly

sudo -H pip3 install setuptools toml pyros-genmsg packaging


# PX4/Firmware
if [ ! -e ${PX4_FIRMWARE_GIT_DIR} ]
then
    cd ${GIT_DIR}
    git clone --recursive \
        --branch ${DRONES_BRANCH} \
        https://github.com/iDROAtLeeds/px4_firmware.git \
        ${PX4_FIRMWARE_GIT_DIR}
fi

# Note: this is built outside the workspace as we need to control the build
# more closely that the package build allows.


# PX4/sitl_gazebo.
# Dependent on mavlink headers.
# Clone repo.
if [ ! -e ${PX4_SITL_GAZEBO_GIT_DIR} ]
then
    mkdir -p ${PX4_GIT_DIR}
    cd ${PX4_GIT_DIR}
    git clone \
        --recursive \
        --branch ${DRONES_BRANCH} \
        https://github.com/iDROAtLeeds/px4_sitl_gazebo.git \
        ${PX4_SITL_GAZEBO_GIT_DIR}
fi

# Link to workspace dir.
mkdir -p ${COLCON_SRC_DIR}
cd ${COLCON_SRC_DIR}
ln -sf ${PX4_SITL_GAZEBO_GIT_DIR}


# PX4/px4_msgs
if [ ! -e ${PX4_MSGS_GIT_DIR} ]
then
    mkdir -p ${PX4_GIT_DIR}
    cd ${PX4_GIT_DIR}
    git clone --branch ${DRONES_BRANCH} \
        https://github.com/iDROAtLeeds/px4_msgs.git \
        ${PX4_MSGS_GIT_DIR}
fi

# Link to workspace dir.
mkdir -p ${COLCON_SRC_DIR}
cd ${COLCON_SRC_DIR}
ln -sf ${PX4_MSGS_GIT_DIR}


# PX4/px4_ros_com
# The build fails when symbolic links are used so clone directly into the workspace dir.
mkdir -p ${COLCON_SRC_DIR}
if [ ! -e ${PX4_ROS_COM_DIR} ]
then
    cd ${COLCON_SRC_DIR}
    git clone \
        --branch ${DRONES_BRANCH} \
		https://github.com/iDROAtLeeds/px4_ros_com.git \
        ${PX4_ROS_COM_DIR}
fi
