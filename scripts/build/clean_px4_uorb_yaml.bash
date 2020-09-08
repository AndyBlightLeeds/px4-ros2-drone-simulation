#!/bin/bash
# Clean the parts of the bulid needed to pick up changes to the uORB YAML file.
set -e

. ../project_vars.bash

# Clean client code.
if [ -e ${PX4_FIRMWARE_GIT_DIR} ]
then
    cd ${PX4_FIRMWARE_GIT_DIR}
    rm -rf ./build/px4_sitl_rtps/src/modules/micrortps_bridge/
fi
# Clean agent code.
if [ -e ${COLCON_WS_DIR} ]
then
    cd ${COLCON_WS_DIR}
    rm -rf ./src/px4_ros_com/src/micrortps_agent/
    rm -rf ./build/px4_ros_com/
    rm -rf ./install/share/ament_index/resource_index/parent_prefix_path/px4_ros_com
    rm -rf ./install/share/ament_index/resource_index/packages/px4_ros_com
    rm -rf ./install/share/ament_index/resource_index/package_run_dependencies/px4_ros_com
    rm -rf ./install/share/colcon-core/packages/px4_ros_com
    rm -rf ./install/share/px4_ros_com
    rm -rf ./install/include/px4_ros_com
    rm -rf ./install/lib/px4_ros_com
fi

echo "Cleaned uRPTS files. Now rebuild using:"
echo "./px4_firmware.bash"
echo "./px4_ros_com.bash"
