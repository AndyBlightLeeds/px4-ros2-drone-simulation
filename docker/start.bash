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
    # Setup X window for the container to use.
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]
    then
        xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
        if [ ! -z "$xauth_list" ]
        then
            echo $xauth_list | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi

    docker container run \
        --detach \
        --tty \
        --name ${SIMULATION_CONTAINER_NAME} \
        --volume ${repo_root_dir}:/home/build/code \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --gpus all \
        ${DOCKER_HUB_USER_NAME}/${SIMULATION_IMAGE}:${SIMULATION_TAG} &> /dev/null
    echo "Container '${SIMULATION_CONTAINER_NAME}' running."
fi
