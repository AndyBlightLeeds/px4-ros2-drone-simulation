# px4-ros2-drone-simulation

This repo allows the user to set up, build and run a simulation using:

* An Iris done running PX4 firmware.
* The PX4-FastRTPS bridge to communicate with a ROS2 program that controls
the Iris drone.
* The Iris drone is simulated using Gazebo.

The intention is to eventually control multiple drones simulataneously but
getting one drone to fly is the first step.

The versions of software used are:

* Ubuntu 18.04LTS.
* Gazebo 11.
* ROS2 Eloquent.
* The PX4 repos Firware, px4_msgs, px4_ros_com, sitl_gazebo.  Git branches
of these repos are used to fix revisions.

Bash scripts are used to install, build and run the simulation code. Bash
scripts were chosen as they can be used directly on a PC, a small companion
computer mounted on a drone such as a Raspberry Pi or in a docker container.
The scripts are split into three directories, install, build and run.

## Installation

TODO - Add install instructions.

## Build

TODO - Add build instructions.

## Running the simulation

TODO - Add run instructions.

## References

Inspriation for this work was taken from the following repos:

* <https://github.com/osrf/drone_demo> Explained how to simulate a drone in
Gazebo (useful but I couldn't run the tests).
* TODO add more.

## TODO

Fix paths to repos.
Check the install and build in a docker image to ensure that everything needed
is in the scripts.
Verify functionality.

### Docker

* Write scripts to create and run docker.
* Test install scripts etc. in a docker.
* Verify simulation can be run from the docker.
