#!/bin/bash
# Start the docker image.
# set -x

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
repo_root_dir=${scripts_dir}/..
. ${scripts_dir}/vars.bash

docker container inspect ${SIMULATION_CONTAINER_NAME} &> /dev/null
if [ $? == 0 ]
then
    # Container exists.
    if [ "$( docker container inspect -f '{{.State.Status}}' ${SIMULATION_CONTAINER_NAME} )" == "running" ]
    then
        # Container is running.
        echo "Container '${SIMULATION_CONTAINER_NAME}' is already running."
    else
        # Container exists but is not running.
        docker start ${SIMULATION_CONTAINER_NAME} &> /dev/null
        echo "Container '${SIMULATION_CONTAINER_NAME}' started."
    fi
else
    # Container does not exist.
    docker run -dt \
        --name ${SIMULATION_CONTAINER_NAME} \
        --restart unless-stopped \
        -v ${repo_root_dir}:/home/build/code \
        ${SIMULATION_IMAGE}:${SIMULATION_TAG}
    echo "Container '${SIMULATION_CONTAINER_NAME}' running."
fi
