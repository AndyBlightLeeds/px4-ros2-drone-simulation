#!/bin/bash
# Remove the docker container and image.
# set -e

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${scripts_dir}/vars.bash

# Delete the image.
image_tag="${DOCKER_HUB_USER_NAME}/${SIMULATION_IMAGE}:${SIMULATION_TAG}"
image_id=`docker image inspect -f '{{.Id}}' ${image_tag}`
if [ $? == 0 ]
then
    docker image rm -f ${image_id}
    echo "Docker image '${image_tag}' removed."
fi
