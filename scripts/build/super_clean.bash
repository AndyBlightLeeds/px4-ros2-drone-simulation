#!/bin/bash
# Clean the parts of the bulid needed to pick up changes to the uORB YAML file.
set -ex

. ../project_vars.bash

# Clean the firmware build.
cd ${PX4_GIT_DIR}/Firmware
git reset --hard
git clean -dfx
git submodule foreach "git reset --hard; git clean -dfx"


# Clean the px4_msgs repo.
cd ${COLCON_WS_DIR}/src/px4_msgs
git reset --hard
git clean -dfx

# Clean the px4_ros_com repo.
cd ${COLCON_WS_DIR}/src/px4_ros_com
git reset --hard
git clean -dfx

# Clean the various build artifacts manually as there is no colcon clean.
cd ${COLCON_WS_DIR}
rm -rf build/ install/ log/

echo "Cleaned all PX4 code."
