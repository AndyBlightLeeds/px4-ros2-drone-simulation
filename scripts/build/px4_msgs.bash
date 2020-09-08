#!/bin/bash
# Builds the PX4 messages package.
set -e

. ../project_vars.bash

# Build.
cd ${COLCON_WS_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
colcon build --merge-install --packages-select px4_msgs

echo "Build took $SECONDS seconds."
