# px4-ros2-drone-simulation

![Build and install.](https://github.com/AndyBlightLeeds/px4-ros2-drone-simulation/workflows/Build%20and%20install./badge.svg)

This repo allows the user to set up, build and run a simulation using:

* An Iris drone running PX4 firmware.
* The PX4-FastRTPS bridge to communicate with a ROS2 program that controls
the Iris drone.
* The Iris drone is simulated using Gazebo.

The versions of software used are:

* Ubuntu 18.04LTS.
* Gazebo 11.
* ROS2 Eloquent.
* The PX4 repos Firware, px4_msgs, px4_ros_com, sitl_gazebo.  Git branches
of these repos are used to fix revisions.  There is also a small fix on the
PX4 Firmware branch.

## __IMPORTANT NOTES__

__Running the Gazebo server and client on a PC requires a graphics card
with at least 4GB of RAM.__  I tried an old 1GB card which worked but was
painfully slow.  I then used a laptop with an NVidia M2000 4GB card and it
worked well.

## Install and run

Docker install and run instructions can be found in `docker/README.md`.

Native install and run instructions can be found in `scripts/README.md`.

## References

Most of the inspiration for this work was taken from
<https://github.com/osrf/drone_demo> that explained how to simulate a drone in
Gazebo.  There are also countless uses of the following sites:

* <https://dev.px4.io/v1.10/en/>
* <https://discuss.px4.io/>

## Status

* Drone::TestAutoModes works.
* Drone::TestMission does not take off using setpoint_position messages.
I must be missing something...
* TODO. #1 Move the tests into a proper test harness.
