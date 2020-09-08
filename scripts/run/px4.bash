# From launch_drone_ros2.py.
# The /tmp/px4 dir contains everthing for that drone.
. ./params.bash
. /opt/ros/eloquent/setup.bash

# Remove any left over files.
rm -rf /tmp/px4*

# Create PX4 rootfs directory and populate it with necessary files.
if [ ! -e ${ROOTFS} ]
then
    mkdir -p ${ROOTFS}
    cd ${ROOTFS}
    cp -aR ${PX4_FIRMWARE_GIT_DIR}/ROMFS .
    cp -aR ${PX4_FIRMWARE_GIT_DIR}/build/px4_sitl_rtps/bin .
fi

# Start PX4 instance using the URDF file.
cd ${ROOTFS}
PX4_SIM_MODEL=iris \
bin/px4 ${ROOTFS}/ROMFS/px4fmu_common \
    -s ${ROOTFS}/ROMFS/px4fmu_common/init.d-posix/rcS \
    -i 0
