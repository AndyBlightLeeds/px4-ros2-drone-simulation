#!/bin/bash
# Start up a single drone (iris 0).

set -e

. ./params.bash

LOG_DIR=logs
mkdir -p ${LOG_DIR}

# Start the process with all output piped to a log file.
function new_process {
    echo "Starting $1"
    ./$1.bash $2 &> ${LOG_DIR}/$1.log &
    # Sleep to allow the process time to start up.
    sleep 1
}

# Gazebo server and client.
new_process gzserver
new_process gzclient

# Spawn the model.
new_process spawn_model

# Start PX4 and uRTPS client.
new_process px4 -d
new_process micrortps_client

# uRTPS agent.
new_process micrortps_agent

echo "Done"

