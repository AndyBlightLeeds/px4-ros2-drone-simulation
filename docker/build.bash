#!/bin/bash
# Build the docker image.
set -e

. ./vars.bash

cp -rf ../scripts .
docker build -t ${CONTAINER_NAME}:${CONTAINER_VERSION} .
# rm -rf scripts
