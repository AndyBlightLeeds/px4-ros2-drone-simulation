# px4-ros2-drone-simulation

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
of these repos are used to fix revisions.

## IMPORTANT NOTE

Running the Gazebo server and client on a PC requires a decent graphics card.
The simplest way to judge if it is good enough is to state that you need a card
with at least 4GB of RAM.  I tried an old 1GB card which did work but was
painfully slow.  I then used a laptop with an NVidia M2000 4GB card and it
worked well.

## Docker install and run

Docker install and run instructions can be found in `docker/README.md`.

## Native install and run

Bash scripts are used to install, build and run the simulation code. Bash
scripts were chosen as they can be used directly on a PC, a small companion
computer mounted on a drone (e.g. a Raspberry Pi) or in a docker container.
The scripts are split into three directories, install, build and run.

To install and build the simulation code on your PC, run the following 
commands:

```text
cd scripts/install
./install_all.bash
cd ../build
./build_all.bash
```

The install and build scripts took around 30 minutes on my PC.  Then they 
have completed, start Gazebo, a simulated Iris drone and the FastRTPS ROS2
bridge using the commands:

```text
cd scripts/run
./auto_start.bash
```

You should see the Gazebo client start with an Iris drone on the ground. 
To start the drone test software, run the following commands:

```text
cd ~/px4_drone_simulation_ws/
. ./install/setup.bash
./build/drone/drone
```

You should see the drone execute the test missions defined in `drone.cpp`.

### Debugging

I use Kubuntu instead of Ubuntu as I prefer the user interface.  The script 
`scripts/run/konsole_start.bash` does the same as `auto_start.bash` but opens 
a new Konsole tab for each shell.  This make debugging much easier as you have
an interactive shell open on the PX4 instance and can see messages from each 
process in a different tab.

## References

Most of the inspiration for this work was taken from
<https://github.com/osrf/drone_demo> that explained how to simulate a drone in
Gazebo.  There are also countless uses of the following sites:

* <https://dev.px4.io/v1.10/en/>
* <https://discuss.px4.io/>

## Status

* Drone::TestAutoModes works.
* Drone::TestMission arms but does not take off. This uses a waypont mission but something is broken.
