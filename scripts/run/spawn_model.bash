#!/bin/bash
set -e

. ./params.bash
. ~/gazebo_learning_ws/install/setup.bash

./spawn_model.py
