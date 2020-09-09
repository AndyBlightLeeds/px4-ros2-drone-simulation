#!/bin/bash
# Installs all repos and sets up workspace.

set -ex

# Install pre-requesites.
# apt-utils make for neater apt installs.
# gpg is needed for adding keys for ROS packages etc.
# curl is used to fetch time zone info.
# wget is used in scripts.
sudo apt-get update
sudo apt-get -y --no-install-recommends install \
    apt-utils \
    gpg \
    curl \
    wget \
    dirmngr
# This sorts out the problem with tzdata needing a user response.
sudo ln -snf /usr/share/zoneinfo/$(curl https://ipapi.co/timezone) /etc/localtime

./install_ros2.bash
./install_ros2_gazebo11.bash
./install_fast_rtps.bash
./install_px4_repos.bash
./install_drone_packages.bash
