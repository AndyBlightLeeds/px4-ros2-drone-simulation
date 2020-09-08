#!/bin/bash
# Install and setup Gazebo 9.
# Based on http://gazebosim.org/tutorials?tut=ros2_installing
set -e

. ../project_vars.bash

# Abort if ROS2 not installed.
if [ ! -e /opt/ros/${ROS2_DISTRO}/setup.bash ]
then
    echo "Could not find ROS2 ${ROS2_DISTRO}.  Aborting"
    exit 1
fi

if [ ! -e /etc/apt/sources.list.d/gazebo-stable.list ]
then
    # Install the key.
    wget  http://packages.osrfoundation.org/gazebo.key
    sudo apt-key add gazebo.key
    # setup sources.list
    command="echo \"deb [arch=amd64] http://packages.osrfoundation.org/gazebo/ubuntu-stable "
    command+=${UBUNTU_RELEASE}
    command+=" main\" > /etc/apt/sources.list.d/gazebo-stable.list"
    sudo sh -c "${command}"
fi

# Install ROS2 Gazebo11 packages
sudo apt update
sudo apt install -y \
    ros-${ROS2_DISTRO}-gazebo11-dev \
    ros-${ROS2_DISTRO}-gazebo11-msgs \
    ros-${ROS2_DISTRO}-gazebo11-plugins \
    ros-${ROS2_DISTRO}-gazebo11-ros \
    ros-${ROS2_DISTRO}-gazebo11-ros-pkgs \
    gazebo11-doc

# Instructions to user.
echo "To test the installation:"
echo " . /opt/ros/${ROS2_DISTRO}/setup.bash"
echo " gazebo --version"
echo "Should return text with 'version 11.00' in it."
echo "Then try this command"
echo " gazebo --verbose /opt/ros/eloquent/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world"
echo "Open another shell and run these commands:"
echo " . /opt/ros/${ROS2_DISTRO}/setup.bash"
echo " ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{angular: {z: 0.1}}' -1"
echo "You should see the robot rotate on the spot."
