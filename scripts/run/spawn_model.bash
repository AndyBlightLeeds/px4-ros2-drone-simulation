#!/bin/bash
set -e

# Setup environment,
. ./params.bash
export PX4_SITL_GAZEBO_GIT_DIR
. ${COLCON_WS_DIR}/install/setup.bash

# Spawn model.
./spawn_model.py
