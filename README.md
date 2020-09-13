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

Installation and running instructions can be found in `docker/README.md`.

## IMPORTANT NOTE

Running the Gazebo server and client on a PC requires a decent graphics card.
The simplest way to judge if it is good enough is to state that you need a card
with at least 4GB of RAM.  I tried an old 1GB card which did work but was
painfully slow.  I then used a laptop with an NVidia M2000 4GB card and it
worked well.

## References

Most of the inspiration for this work was taken from
<https://github.com/osrf/drone_demo> that explained how to simulate a drone in
Gazebo.  There are also countless uses of the following sites:

* <https://dev.px4.io/v1.10/en/>
* <https://discuss.px4.io/>

## TODO

All done :-)
