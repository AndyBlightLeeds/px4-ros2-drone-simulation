# Docker

This directory provides the user with a method to quickly build and test the
drone code.  Scripts are provided to create the docker image and to start and
stop the container.

NOTE: running the simulation requires a graphics card with 4GB RAM or better.

To set up your PC to use NVidia, please read the section `Set up NVidia`.

## Basic operation

The script `build.bash` creates the docker container, installing all packages
as defined in the `scripts/install` directory.  Do this just once!

To start the container, use `start.bash`.  This script starts the container
and leaves it running until `stop.bash` is called.

When the container is running, you can get a Bash user prompt using
`connect.bash`.

At the new Bash prompt, enter the following:

```text
cd code/scripts/run/
./auto_start.bash
```

At this point, you should be able to see the Gazebo client with an Iris drone
on the ground.


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

NOTE: You may need to use another version of CUDA. 9.0 worked for me but 11.0
is the current version.

### Running the ros_melodic_nvdia docker image

This is a bare bones version of the info in section 1.3 here:
<http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration>

I have included it in this repo as is a quick way to verify that you have
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

1. The code repo is outside the docker and is mounted on to `/home/build/`.
1. The docker build is done in two parts for speed reasons.
   1. The first image built is ROS2 Eloquent desktop.  This build takes about
   15 minutes even on a fast PC so it is worth spending the extra time getting
   it built and out of the way.  This image can also be used for other
   projects.
   2. The second build uses the first image as a starting point. First, the
   various packages and code for the simulation are installed and then all the
   code is built.  This takes another 15 minutes or so to complete.  It is a
   big build!

## Troubleshooting

### PX4 terminates when run in docker

```text
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2020-09-11/14_32_36.ulg
INFO  [logger] Opened full log file: ./log/2020-09-11/14_32_36.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> Exiting NOW.
```

This should not exit.
