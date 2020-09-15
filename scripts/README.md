# Native install, build and run

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

The install and build scripts took around 30 minutes on my PC.  When they
have finished, start Gazebo, a simulated Iris drone and the FastRTPS ROS2
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

## Debugging

I use Kubuntu instead of Ubuntu as I prefer the user interface.  The script
`scripts/run/konsole_start.bash` does the same as `auto_start.bash` but opens
a new Konsole tab for each shell.  This make debugging much easier as you can
see messages from each process in a different tab.
