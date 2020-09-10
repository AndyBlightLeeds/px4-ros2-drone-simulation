#!/bin/bash
# Give the user a prompt on the docker container.
# Only works for one container running.
set -e

scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${scripts_dir}/vars.bash
docker exec -it -w /home/build ${SIMULATION_CONTAINER_NAME} /bin/bash
