#!/bin/bash
set -e

. ./params.bash
. /opt/ros/eloquent/setup.bash
. /usr/share/gazebo-11/setup.sh
# This adds the paths for the mavlink_sitl models etc,
. ${COLCON_WS_DIR}/install/share/mavlink_sitl_gazebo/setup.sh

# echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
# echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
# echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}"
# echo "GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}"
# echo "GAZEBO_MODEL_DATABASE_URI=${GAZEBO_MODEL_DATABASE_URI}"
# echo
gzclient --verbose

