#!/bin/bash
# Start the docker image.
set -e

. ./vars.bash

# TODO USE realpath
cd ..
docker run -dt \
    --name robot_env \
    --restart unless-stopped \
    -v `pwd`:/home/user \
    ${CONTAINER_NAME}
