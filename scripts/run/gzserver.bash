#!/bin/bash
set -e

. ./params.bash

. /opt/ros/${ROS2_DISTRO}/setup.bash
. /usr/share/gazebo-11/setup.sh

# This is needed for libphysics_msgs.so
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${COLCON_WS_DIR}/build/mavlink_sitl_gazebo:

# echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
# echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
# echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
# echo "GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}"
# echo "GAZEBO_MODEL_DATABASE_URI=${GAZEBO_MODEL_DATABASE_URI}"
# echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"

gzserver --verbose --physics=ode \
    --server-plugin libgazebo_ros_factory.so \
    worlds/empty.world
