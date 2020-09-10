#!/bin/bash
# Start the docker image.
# set -x

. ./vars.bash

# TODO USE realpath

docker container ls -q --filter "name=${SIMULATION_CONTAINER_NAME}" &> /dev/null
if [ $? == 0 ]
then
    # Container exists.
    if [ "$( docker container inspect -f '{{.State.Status}}' ${SIMULATION_CONTAINER_NAME} )" == "running" ]
    then
        # Container is running.
        echo "Container '${SIMULATION_CONTAINER_NAME}' is already running."
    else
        docker start ${SIMULATION_CONTAINER_NAME} &> /dev/null
        echo "Container '${SIMULATION_CONTAINER_NAME}' started."
    fi
else
    # Container does not exist so run it.
    docker run -dt \
        --name ${SIMULATION_CONTAINER_NAME} \
        --restart unless-stopped \
        -v `pwd`:/home/user \
        ${SIMULATION_IMAGE}:${SIMULATION_TAG}
    echo "Container '${SIMULATION_CONTAINER_NAME}' running."
fi
