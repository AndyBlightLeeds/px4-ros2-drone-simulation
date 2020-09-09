#!/bin/bash
# Give the user a prompt on the docker container.
set -e

. ./vars.bash

docker exec -it ${CONTAINER_NAME} bash
