#!/bin/bash
# Stop the docker image.

. ./vars.bash

docker container ls --filter "name=${SIMULATION_CONTAINER_NAME}"
if [ $? == 1 ]
then
    docker stop ${SIMULATION_CONTAINER_NAME}
fi

echo "Docker '${SIMULATION_CONTAINER_NAME}' stopped"
