# Docker

This directory provides the user with a method to quickly build and test the
drone code.  Scripts are provided to create the docker image and to start and
stop the container.

NOTE: running the simulation requires a graphics card with 4GB RAM or better.

## Basic operation

The script `build.bash` builds the docker container.  Do this just once!

To start the container, use `start.bash`.  This script starts the container
and leaves it running until `stop.bash` is called.

Once the container is started, you can get a bash user prompt using
`connect.bash`.

## NOTES

1. The code repo is outside the docker and is mounted on to `/home/user/`.
1. The docker build is done in two parts for speed reasons.
   1. The first image built is ROS2 Eloquent desktop.  This build takes about
   15 minutes even on a fast PC so it is worth spending the extra time getting
   it built and out of the way.  This image can also be used for other
   projects.
   2. The second build uses the first image as a starting point and so takes
   much less time to build.
