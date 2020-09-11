#!/bin/bash
# Installs all repos and sets up workspace.
set -e

# Install pre-requesites.
# apt-utils make for neater apt installs.
# gnupg2 and dirmngr are needed for adding keys for ROS packages etc.
# curl is used to fetch time zone info.
# wget is used in scripts.
sudo apt-get update
sudo apt-get -y --no-install-recommends install \
    apt-utils \
    gnupg2 dirmngr\
    curl \
    wget

# Now install the scripts.
./install_ros2.bash
./install_ros2_gazebo11.bash
./install_fast_rtps.bash
./install_px4_repos.bash
./install_drone_packages.bash
