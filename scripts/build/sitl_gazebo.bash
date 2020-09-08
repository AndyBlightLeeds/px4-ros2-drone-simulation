#!/bin/bash
# Builds the PX4/sitl_gazebo package.
set -e

. ../project_vars.bash

# Needs to have MAVLink headers
#  mavlink/v1.0/mavlink_types.h and
#  mavlink/v2.0/mavlink_types.h
# installed on one of these paths:
#  ${CMAKE_SOURCE_DIR}/mavlink/
#  ../../mavlink/include
#  ../mavlink/include
#  /usr/include
#  /usr/local/include
#  /opt/ros/${ROS_DISTRO}/include
# The simplest thing to do to meet this requirement is to install
# the package ros-eloquent-mavlink. See install_px4_repos.bash.

# Build.
cd ${COLCON_WS_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
colcon build --merge-install --packages-select mavlink_sitl_gazebo

echo "Build took $SECONDS seconds."
