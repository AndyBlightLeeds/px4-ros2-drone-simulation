#!/bin/bash
set -e

. ./params.bash

# Note: ports are opposite to that used for the PX4 client.
cd ${COLCON_WS_DIR}
build/px4_ros_com/micrortps_agent -t UDP \
-r ${MICRO_RTPS_SEND_PORT} \
-s ${MICRO_RTPS_RECEIVE_PORT}
