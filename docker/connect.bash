#!/bin/bash
# Give the user a prompt on the docker container.
# Only works for one container running.
set -e

. ./vars.bash

docker exec -it ${SIMULATION_CONTAINER_NAME} /bin/bash
