#!/bin/bash
set -e

. ./params.bash

cd ${ROOTFS}
bin/px4-micrortps_client \
    --instance ${VEHICLE_ID} start -t UDP \
    -r ${MICRO_RTPS_RECEIVE_PORT} \
    -s ${MICRO_RTPS_SEND_PORT}
