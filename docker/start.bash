#!/bin/bash
# Start the docker container.
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
        docker container start ${SIMULATION_CONTAINER_NAME} &> /dev/null
        echo "Container '${SIMULATION_CONTAINER_NAME}' started."
    fi
else
    # Container does not exist.
    docker container run \
        --detach \
        --tty \
        --gpus all \
        --name ${SIMULATION_CONTAINER_NAME} \
        --volume ${repo_root_dir}:/home/build/code \
        ${SIMULATION_IMAGE}:${SIMULATION_TAG}  &> /dev/null
    echo "Container '${SIMULATION_CONTAINER_NAME}' running."
fi
