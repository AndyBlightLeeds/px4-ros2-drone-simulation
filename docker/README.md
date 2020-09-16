# Docker

This directory provides the user with a method to quickly build and test the
drone package in this repo in a docker container.  Scripts are provided to
create the docker image and to start, stop and attach to the docker container.

__IMPORTANT NOTE: running the simulation requires a graphics card with
4GB RAM or better.__

To set up your PC to use a NVidia graphics card, please read the section
`Set up NVidia`.  Docker supports other graphics cards but these have not been
tested or documented.

## Basic operation

The script `~/code/docker/build.bash` creates the docker container, installing all
packages as defined in the `~/code/scripts/install` directory.  Do this just once!
This process took about 30 minutes on my PC, so get on with something else
while the image is built.

To start the container, use `~/code/docker/start.bash`.  This script starts the
container and leaves it running until `~/code/docker/stop.bash` is called.

When the container is running, you can get a Bash user prompt attached to the
container using `~/code/docker/attach.bash`.

After attaching to the container for the first time, build the drone package
as follows:

```text
cd ~/code/scripts/build
./drone.bash
```

Once this is done, you are ready to start the simulation. At the Bash prompt,
enter the following:

```text
cd ~/code/scripts/run
./auto_start.bash
```

At this point, you should be able to see the Gazebo client with an Iris drone
on the ground.  Using `ps` on the shell should show something like this:

```text
$ ps x
  PID TTY      STAT   TIME COMMAND
    1 pts/0    Ss+    0:00 /bin/bash
   10 pts/1    Ss     0:00 /bin/bash
 1166 pts/1    S      0:00 /bin/bash ./gzserver.bash
 1196 pts/1    Sl    14:34 gzserver --verbose --physics=ode --server-plugin libgazebo_ros_factory.so worlds/empty.world
 1240 pts/1    S      0:00 /bin/bash ./gzclient.bash
 1270 pts/1    Sl     9:16 gzclient --verbose
 1344 pts/1    S      0:00 /bin/bash ./auto_start.bash
 1378 pts/1    Sl     4:10 bin/px4 /tmp/px4/ROMFS/px4fmu_common -s /tmp/px4/ROMFS/px4fmu_common/init.d-posix/rcS -i 0 -d
 1634 pts/1    S      0:00 /bin/bash ./micrortps_agent.bash
 1636 pts/1    Sl    20:04 build/px4_ros_com/micrortps_agent -t UDP -r 2020 -s 2019
 1899 pts/2    Ss+    0:00 /bin/bash
 2299 pts/1    R+     0:00 ps x

```

This shows that there are four processes running: Gazebo server and client,
one instance of PX4 and the MicroRTPS agent.  These four process are the
minimum needed to run the drone test software.

To start the drone test software, run the following commands:

```text
cd ~/px4_drone_simulation_ws/
. ./install/setup.bash
./build/drone/drone
```

You should see the drone execute the test missions defined in `drone.cpp`.

## Set up NVidia

NOTE: The docker image is setup for NVidia and uses the `nvidia-docker2`
software.

On your PC, install the NVidia container toolkit.

```text
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

Then test using:

```bash
docker run --gpus all nvidia/cuda:9.0-base nvidia-smi
```

NOTE: You may need to use another version of CUDA. 9.0 worked for me but 11.0
is the current version, e.g.

```bash
docker run --gpus all nvidia/cuda:11.0-base nvidia-smi
```

When the docker has been downloaded and installed, you should see something
like this:

```text
Fri Sep 11 09:18:06 2020
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 440.100      Driver Version: 440.100      CUDA Version: 10.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  Quadro M2000M       Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   44C    P0    N/A /  N/A |    663MiB /  4043MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
+-----------------------------------------------------------------------------+
```

### Running the ros_melodic_nvdia docker image

This is a bare bones version of the info in section 1.3 here:
<http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration>

I have included it in this repo as a quick way to verify that you have
everything setup correctly on your PC to run the main docker image.

Run the following commands:

```bash
cd docker/ros-melodic-nvidia-test
./build.bash
./run.bash
```

In the new shell on the container, run the command:

```bash
roscore > /dev/null & rosrun rviz rviz
```

RViz should start up.

## NOTES

1. The code repo is outside the docker and is mounted on to `/home/build/code`.
1. The docker build is done in two parts.
   1. The first image built is ROS2 Eloquent desktop.  This build takes about
   15 minutes even on a fast PC so it is worth spending the extra time getting
   it built and out of the way.  This image can also be used for other
   projects.
   1. The second build uses the first image as a starting point. First, the
   various packages and code for the simulation are installed and then all the
   code is built.  This takes another 15 minutes or so to complete.  It is a
   big build!
