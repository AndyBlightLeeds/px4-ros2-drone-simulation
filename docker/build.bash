#!/bin/bash
# Build the docker image.
set -e

. ./vars.bash

mkdir -p context
cd context
cp -rf ../../scripts .
docker build -t ${CONTAINER_NAME}:${CONTAINER_VERSION}
# cd ..
# rm -rf context
