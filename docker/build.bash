#!/bin/bash
# Build the docker image.
# Not using set -e as `docker inspect` can legitimately fail.

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

# Tidy up
rm -rf scripts
