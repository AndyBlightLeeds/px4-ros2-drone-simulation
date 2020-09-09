#!/bin/bash
# Install dependencies and setup the drone repos.
set -e

. ../project_vars.bash

# Set up liinks to repos.
mkdir -p ${COLCON_SRC_DIR}
cd ${COLCON_SRC_DIR}
ln -sf ${DRONE_GIT_DIR}
