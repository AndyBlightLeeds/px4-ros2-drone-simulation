#!/bin/bash
# Build the docker image.
set -e

. ./vars.bash

cp -rf ../scripts .
docker inspect --type=image ${ROS2_DESKTOP_IMAGE}:${ROS2_DESKTOP_TAG}
if [ $? == 1 ]
then
    docker build \
        -t ${ROS2_DESKTOP_IMAGE}:${ROS2_DESKTOP_TAG} \
        -f ${ROS2_DESKTOP_IMAGE}.dockerfile \
        .
fi

docker inspect --type=image ${SIMULATION_IMAGE}:${SIMULATION_TAG}
if [ $? == 1 ]
then
    docker build \
        -t ${SIMULATION_IMAGE}:${SIMULATION_TAG} \
        -f ${SIMULATION_IMAGE}.dockerfile \
        .
fi
# rm -rf scripts
