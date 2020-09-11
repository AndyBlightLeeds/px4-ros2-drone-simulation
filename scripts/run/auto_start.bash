#!/bin/bash
# Start up a single drone (iris 0).

set -e

. ./params.bash

# There is a `sleep 1` between each command to allow it time to finish.
# This is needed as
# Gazebo server and client.
./gzserver.bash &
sleep 1
./gzclient.bash &
sleep 1

# Spawn the model. This terminates after being run.
./spawn_model.bash

# Start PX4.
# TODO It would be good to keep this session interactive as I have
# found it useful to be able to run PX4 commands when debugging.
./px4.bash &
# Start microRTPS client on PX4. This terminates after being run.
./micrortps_client.bash

# uRTPS agent.
./micrortps_agent.bash &

echo "Done"

