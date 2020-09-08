#!/bin/bash
# Install and setup the ROS2 px4_ros_com package.
# Based on the process on this page:
# https://dev.px4.io/master/en/middleware/micrortps.html
#
# Dependencies.
# ./px4_firmware.bash
# ./px4_msgs.bash
set -e

. ../project_vars.bash

if [ ! -e /usr/local/bin/fastrtpsgen ]
then
	echo "ERROR: fastrtpsgen is not installed."
	echo "Please run:"
	echo
	echo "cd ../install"
	echo "install_fast_rtps.bash"
	echo
	exit 1
fi

# Simulate what the build server does to the uORB YAML file (single source of truth).
# The step in the JenkinsFile are:
#   sh('./msg/tools/uorb_to_ros_rtps_ids.py -i msg/tools/uorb_rtps_message_ids.yaml -o px4_ros_com/templates/uorb_rtps_message_ids.yaml')
${PX4_FIRMWARE_GIT_DIR}/msg/tools/uorb_to_ros_rtps_ids.py \
	-i ${PX4_FIRMWARE_GIT_DIR}/msg/tools/uorb_rtps_message_ids.yaml \
	-o ${PX4_ROS_COM_DIR}/templates/uorb_rtps_message_ids.yaml

# Build the ROS2 workspace.
cd ${COLCON_WS_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
# NOTE: this command does not pick up changes to the YAML file.
# It works fine for first time builds.
# To ensure changes are picked up, use the file clean_px4_uorb_yaml.bash.
colcon build --merge-install --packages-select px4_ros_com


echo "Build took $SECONDS seconds."
echo
echo "The mircortps_agent can be run using:"
echo "${WORKSPACE_DIR_2}/install/px4_ros_com/bin/micrortps_agent -t UDP"
echo
