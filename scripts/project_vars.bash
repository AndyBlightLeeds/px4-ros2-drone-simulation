# Source this file in other shell scripts.

UBUNTU_RELEASE=bionic
# Export colcon workspace directories.
PROJECT_DIR=${HOME}
COLCON_WS_DIR=${PROJECT_DIR}/drone_simulation_ws
COLCON_SRC_DIR=${COLCON_WS_DIR}/src
# Export git and source directories.
GIT_DIR=${HOME}/git
PX4_GIT_DIR=${GIT_DIR}/idro
PX4_FIRMWARE_GIT_DIR=${PX4_GIT_DIR}/px4_firmware
PX4_ROS_COM_DIR=${COLCON_SRC_DIR}/px4_ros_com
PX4_SITL_GAZEBO_GIT_DIR=${PX4_GIT_DIR}/px4_sitl_gazebo
PX4_MSGS_GIT_DIR=${PX4_GIT_DIR}/px4_msgs
OSRF_GIT_DIR=${GIT_DIR}/osrf
PROJECT_REPO_DIR=${GIT_DIR}/ros2-px4-drone-simulation
DRONE_MSGS_GIT_DIR=${PROJECT_REPO_DIR}/drone_msgs
LEEDS_PUMP_GIT_DIR=${PROJECT_REPO_DIR}/leeds_pump
DRONE_GIT_DIR=${PROJECT_REPO_DIR}/drone
# Needed during builds.
ROS_PYTHON_VERSION=3
ROS2_DISTRO=eloquent
# PX4 repo branch
DRONES_BRANCH=drones-1
