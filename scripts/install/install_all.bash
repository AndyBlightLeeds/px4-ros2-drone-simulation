#!/bin/bash
# Installs all repos and sets up workspace.

set -ex

./install_ros2.bash
./install_ros2_gazebo11.bash
./install_fast_rtps.bash
./install_px4_repos.bash
./install_drone_packages.bash
