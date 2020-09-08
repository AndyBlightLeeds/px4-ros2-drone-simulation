#!/bin/bash
# Sets up the packages as required and builds them.
set -e

. ../project_vars.bash

# Build px4 firmware as we build it differently to the standard colcon package.
./px4_firmware.bash

# Now build the rest.
cd ${COLCON_WS_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
colcon build --merge-install --packages-select px4_msgs

echo "Build took $SECONDS seconds."
