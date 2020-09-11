#!/bin/bash
# Start up a single drone (iris 0).

set -e

. ./params.bash

# There is a `sleep 1` between each command to allow it time to finish.
# This is needed as each process takes a while to get started.

# Gazebo server and client.
./gzserver.bash &> gzserver.log &
sleep 1
./gzclient.bash &> gzclient.log &
sleep 1

# Spawn the model. This terminates after being run.
./spawn_model.bash &> spawn_model.log

# Start PX4.
# TODO It would be good to keep this session interactive as I have
# found it useful to be able to run PX4 commands when debugging.
./px4.bash &> px4.log &
sleep 1
# Start microRTPS client on PX4. This terminates after being run.
./micrortps_client.bash &> gzserver.log &
sleep 1

# uRTPS agent.
./micrortps_agent.bash &> gzserver.log &
sleep 1

echo "Done"

