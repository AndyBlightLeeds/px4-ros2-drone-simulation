#!/bin/bash
# Start up a single drone (iris 0).
# konsole start up based on
# https://superuser.com/questions/682850/open-new-konsole-from-script-executing-command-and-becoming-interactive-on-conc
set -e

. ./params.bash

# Start each process in a new konsole tab.
function start_new_tab {
    echo "Starting $1"
    konsole --new-tab -e /bin/bash --rcfile $1 &
    # Sleep is needed to ensure correct start up order.
    sleep 1
}

# Gazebo server and client.
start_new_tab gzserver.bash
start_new_tab gzclient.bash

# Spawn the model.
start_new_tab spawn_model.bash

# Start PX4 and uRTPS client.
start_new_tab px4.bash
start_new_tab micrortps_client.bash

# uRTPS agent.
start_new_tab micrortps_agent.bash

echo "Done"

