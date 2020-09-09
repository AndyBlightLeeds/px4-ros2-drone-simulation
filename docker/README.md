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

NOTES:
The code repo is outside the docker and is mounted on to `/home/user/`.
