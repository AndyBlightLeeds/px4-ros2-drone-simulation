#!/bin/bash
# Build the docker image.
# set -e  # Not used as docker inspect can fail but we need to carry on.

. ./vars.bash

cp -rf ../scripts .
docker inspect --type=image ${ROS2_DESKTOP_IMAGE}:${ROS2_DESKTOP_TAG}  &> /dev/null
if [ $? == 1 ]
then
    set -e
    docker build \
        -t ${ROS2_DESKTOP_IMAGE}:${ROS2_DESKTOP_TAG} \
        -f ${ROS2_DESKTOP_IMAGE}.dockerfile \
        .
    set +e
fi

docker inspect --type=image ${SIMULATION_IMAGE}:${SIMULATION_TAG} &> /dev/null
if [ $? == 1 ]
then
    set -e
    docker build \
        -t ${SIMULATION_IMAGE}:${SIMULATION_TAG} \
        -f ${SIMULATION_IMAGE}.dockerfile \
        .
    set +e
fi
# rm -rf scripts
