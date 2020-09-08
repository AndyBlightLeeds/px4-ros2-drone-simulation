#!/bin/bash
# Sets up the px4 Firmware package and builds it.
set -ex

. ../project_vars.bash

# These are the original commands in the CMakeLists.txt file for the drone demo.
# set(PX4_TARGET px4_sitl_rtps)
# ExternalProject_Add(firmware
#   GIT_REPOSITORY https://github.com/PX4/Firmware.git
#   GIT_TAG master
#   CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
#   CONFIGURE_COMMAND wget -q https://raw.githubusercontent.com/PX4/Firmware/master/msg/tools/uorb_rtps_message_ids.yaml -O uorb_rtps_message_ids.yaml
#                     && patch --batch uorb_rtps_message_ids.yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/uorb_rtps_message_ids.patch -o ./msg/tools/uorb_rtps_message_ids.yaml
#   BUILD_COMMAND make ${PX4_TARGET}
#   BUILD_IN_SOURCE ON
#   INSTALL_COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX}/share/px4 && rsync -avL --exclude .git --exclude build . build/${PX4_TARGET}/bin ${CMAKE_INSTALL_PREFIX}/share/px4
# )
# ament_environment_hooks(cmake/99_px4_setup.sh.in)
#

# Build the Firmware repo.
PX4_TARGET=px4_sitl_rtps
cd ${PX4_FIRMWARE_GIT_DIR}
git submodule sync --recursive
git submodule update --init --recursive
make ${PX4_TARGET}

# Fake the rest of the original CMakeLists.txt file.
WS_INSTALL_SHARE_DIR=${COLCON_WS_DIR}/install/share
# Tell ament about the package.
mkdir -p ${WS_INSTALL_SHARE_DIR}/ament_index/resource_index/packages
touch ${WS_INSTALL_SHARE_DIR}/ament_index/resource_index/packages/px4

# Copy the px4/bin files into the share directory.
mkdir -p ${WS_INSTALL_SHARE_DIR}/px4/bin
cp -fP ${PX4_FIRMWARE_GIT_DIR}/build/${PX4_TARGET}/bin/* ${WS_INSTALL_SHARE_DIR}/px4/bin

# Another problem is that the function get_package_share_directory('px4')
# returns this directory: /home/andy/gazebo_learning_ws/install/share/px4
# The build system puts the code here: /home/andy/gazebo_learning_ws/install/px4
# So copy the directory manually, Only the ROMFS tree is needed.
cp -rf ${PX4_FIRMWARE_GIT_DIR}/ROMFS ${WS_INSTALL_SHARE_DIR}/px4

echo "Build took $SECONDS seconds."

