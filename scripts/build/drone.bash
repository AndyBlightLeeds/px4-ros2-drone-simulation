#!/bin/bash
set -e

# Source the project variables.
. ../project_vars.bash

# Build.
cd ${COLCON_WS_DIR}
. install/setup.bash
colcon build --merge-install  --packages-select drone
