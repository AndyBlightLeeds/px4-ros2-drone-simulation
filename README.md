# px4-ros2-drone-simulation

This repo allows the user to set up, build and run a simulation using:

* An Iris quad-copter running PX4 firmware.
* The PX4-FastRTPS bridge to communicate with a ROS2 program that controls
the Iris quad-copter.
* The Iris quad-copter is simulated using Gazebo.

The versions of software used are:

* Ubuntu 18.04LTS.
* Gazebo 11.
* ROS2 Eloquent.
* Various PX4 repos using branches to fix revsions.

Bash scripts are used to install, build and run the simulation code. Bash
scripts were chosen as they can be used directly on a PC, a small companion
computer mounted on a drone such as a Raspberry Pi or in a docker container.
The Bash scripts also serve as most of the installation documentation.

The scripts are split into three directories, install, build and run.

## Installation

TODO - Add install instructions.

## Build

TODO - Add build instructions.

## Running the simulation

TODO - Add run instructions.

## TODO

Copy over necessary parts from main project.
Fix paths to repos.
Check the install and build in a docker image to ensure that everything needed
is in the scripts.

### Docker

* Write scripts to create and run docker.
* Test install scripts etc. in a docker.
* Verify simulation can be run from the docker.
