#!/bin/bash
# Install dependencies and setup the drone repos.
set -e

. ../project_vars.bash

# Nothing to download as the scripts that your are using are in the repo.

# Set up links to repos.
mkdir -p ${COLCON_SRC_DIR}
cd ${COLCON_SRC_DIR}

# The drone repo is mounted on ~/code when inside a docker.
if [ -f /.dockerenv ]
then
    ln -sf ~/code drone
else
    # echo "DRONE_GIT_DIR = ${DRONE_GIT_DIR}"
    ln -sf ${DRONE_GIT_DIR}
fi
