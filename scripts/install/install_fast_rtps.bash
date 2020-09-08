#!/bin/bash
# Builds and installs the px4_ros_com package.  Based on:
# https://hub.docker.com/r/px4io/px4-dev-ros2-eloquent/dockerfile

set -e

. ../project_vars.bash
. /opt/ros/${ROS2_DISTRO}/setup.bash

sudo apt install -y libasio-dev libtinyxml2-dev gradle

FAST_RTPS_VERSION=1.9.2
FAST_RTPS_BRANCH=1.9.x
FAST_RTPS_DIR=${GIT_DIR}/fast-rtps
FOONATHAN_DIR=${FAST_RTPS_DIR}/foonathan_memory
FOONATHAN_LIB=/usr/local/lib/libfoonathan_memory-0.6.2.a
FAST_RTPS_GIT_DIR=${FAST_RTPS_DIR}/${FAST_RTPS_VERSION}
FAST_RTPS_LIB=/usr/local/lib/libfastrtps.so
mkdir -p ${FAST_RTPS_DIR}

# Install foonathan_memory from source as it is required to Fast-RTPS >= 1.9
if [ ! -e ${FOONATHAN_LIB} ]
then
    if [ ! -e ${FOONATHAN_DIR} ]
    then
        cd ${FAST_RTPS_DIR}
        git clone https://github.com/eProsima/foonathan_memory_vendor.git ${FOONATHAN_DIR}
    fi
    cd ${FOONATHAN_DIR}
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc) -l$(nproc)
    sudo make install
fi

# Build and install Fast-RTPS library.
if [ ! -e ${FAST_RTPS_LIB}} ]
then
    if [ ! -e ${FAST_RTPS_GIT_DIR} ]
    then
        cd ${FAST_RTPS_DIR}
        git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b ${FAST_RTPS_BRANCH} ${FAST_RTPS_GIT_DIR}
    fi
    cd ${FAST_RTPS_GIT_DIR}
    mkdir -p build
    cd build
    cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
    make -j$(nproc) -l$(nproc)
    sudo make install
fi

# Now build Fast-RTPS-Gen.
FAST_RTPS_GEN_DIR=${FAST_RTPS_DIR}/Fast-RTPS-Gen
if [ ! -e /usr/local/bin/fastrtpsgen ]
then
    if [ ! -e ${FAST_RTPS_GEN_DIR} ]
    then
        cd ${FAST_RTPS_DIR}
        git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.4
    fi
	cd ${FAST_RTPS_GEN_DIR}
	./gradlew assemble
    sudo ./gradlew install
fi

echo "Build took $SECONDS seconds."

