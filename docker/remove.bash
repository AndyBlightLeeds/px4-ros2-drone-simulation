#!/bin/bash
# Remove the docker container and image.
# set -e

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${scripts_dir}/vars.bash

# Remove container.
container_id=`docker container inspect -f '{{.Id}}' ${SIMULATION_CONTAINER_NAME}`
if [ $? == 0 ]
then
    docker container rm -f ${container_id}
    echo "Docker container '${SIMULATION_CONTAINER_NAME}' removed."
fi

# Delete the image.
image_id=`docker image inspect -f '{{.Id}}' ${SIMULATION_IMAGE}:${SIMULATION_TAG}`
if [ $? == 0 ]
then
    docker image rm -f ${image_id}
    echo "Docker image '${SIMULATION_IMAGE}:${SIMULATION_TAG}' removed."
fi