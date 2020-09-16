#!/bin/bash
# Sets up the packages as required and builds them.
set -e

. ../project_vars.bash

# Build px4 firmware as we build it differently to the standard colcon package.
./px4_firmware.bash

# px4_msgs needs to be built before px4_ros_com.
./px4_msgs.bash
./px4_ros_com.bash
./sitl_gazebo.bash

# Only build the drone code if not in a docker.  Mounting a volume during the
# build is not possible, so build the drone code manually.
if [ ! -f /.dockerenv ]
then
    ./drone.bash
fi

echo "Build took $SECONDS seconds."
