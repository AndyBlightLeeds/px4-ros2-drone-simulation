#!/bin/bash
# Start the docker image.
set -e

. ./vars.bash

# TODO USE realpath
docker run -dt \
    --name ${SIMULATION_CONTAINER_NAME} \
    --restart unless-stopped \
    -v `pwd`:/home/user \
    ${SIMULATION_IMAGE}:${SIMULATION_TAG}
