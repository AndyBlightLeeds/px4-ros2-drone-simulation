# Source this file in other shell scripts.

UBUNTU_RELEASE=bionic
# Needed during builds.
ROS_PYTHON_VERSION=3
ROS2_DISTRO=eloquent
# PX4 repo branch
DRONES_BRANCH=px4-drone-demo

# Paths
# This project could be cloned anywhere so make sure the scripts can cope.
# This magic line gets the path to the directory containing this script.
this_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
# echo "this_dir = $this_dir"
THIS_PROJECT_DIR=${this_dir}/..
DRONE_GIT_DIR=${THIS_PROJECT_DIR}/drone

# These directories are created by the build. All are relative to
# PROJECT_DIR that is set to HOME as is it convenient.
PROJECT_DIR=${HOME}

# Colcon workspace directories.
COLCON_WS_DIR=${PROJECT_DIR}/px4_drone_simulation_ws
COLCON_SRC_DIR=${COLCON_WS_DIR}/src

# Git and source directories.
GIT_USER_NAME=andyblightleeds
PX4_GITHUB_URL=https://github.com/${GIT_USER_NAME}
GIT_DIR=${PROJECT_DIR}/git/${GIT_USER_NAME}

# These settings make it easy to swap between different PX4 repos.
PX4_GIT_DIR=${GIT_DIR}
PX4_FIRMWARE_GIT_DIR=${PX4_GIT_DIR}/px4_firmware
PX4_SITL_GAZEBO_GIT_DIR=${PX4_GIT_DIR}/px4_sitl_gazebo
PX4_MSGS_GIT_DIR=${PX4_GIT_DIR}/px4_msgs

# This is an exception. The build fails for some reason if the repo is not
# cloned directly into the workspace.
PX4_ROS_COM_DIR=${COLCON_SRC_DIR}/px4_ros_com
