#!/bin/bash
# Stop the docker container.
set -e

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${scripts_dir}/vars.bash

if [ "$( docker container inspect -f '{{.State.Status}}' ${SIMULATION_CONTAINER_NAME} )" == "running" ]
then
    # Container is running so stop it.
    docker stop ${SIMULATION_CONTAINER_NAME} &> /dev/null
    sudo rm -rf /tmp/.docker.xauth/
    echo "Docker '${SIMULATION_CONTAINER_NAME}' stopped."
else
    echo "Docker '${SIMULATION_CONTAINER_NAME}' already stopped."
fi

