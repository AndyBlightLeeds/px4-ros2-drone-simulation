#!/bin/bash
# Install dependencies and setup the drone repos.
set -e

. ../project_vars.bash

if [ -f /.dockerenv ]
then
    # In a Docker image, link to the externally mounted volume.
    ln -sf ~/code drone
else
    # Set up links to repo.
    mkdir -p ${COLCON_SRC_DIR}
    cd ${COLCON_SRC_DIR}
    ln -sf ${DRONE_GIT_DIR}
fi
