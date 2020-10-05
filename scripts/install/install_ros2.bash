#!/bin/bash
# Install and setup ROS Eloquent.
set -e

. ../project_vars.bash

if [ ! -e /etc/apt/sources.list.d/ros2-latest.list ]
then
    # setup ros2 keys
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    # setup sources.list
    command="echo \"deb [arch=amd64] http://packages.ros.org/ros2/ubuntu "
    command+=${UBUNTU_RELEASE}
    command+=" main\" > /etc/apt/sources.list.d/ros2-latest.list"
    sudo sh -c "${command}"
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pytest \
    python3-rosdep \
    python3-rosdep-modules \
    python3-rospkg \
    python3-rosdistro \
    python3-setuptools \
    python3-vcstool \
    python3-catkin-pkg \
    flake8 \
    ros-${ROS2_DISTRO}-desktop \
    ros-${ROS2_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS2_DISTRO}-rosidl-generator-dds-idl

# The following are recommended.  gcc might be useful!
#   gcc build-essential fakeroot libalgorithm-merge-perl fonts-liberation
#   alsa-ucm-conf alsa-topology-conf at-spi2-core libaacs0 manpages manpages-dev
#   libpam-cap libcfitsio-doc dmsetup libfile-fcntllock-perl
#   liblocale-gettext-perl libexif-doc proj-bin libgdk-pixbuf2.0-bin
#   xdg-user-dirs libgphoto2-l10n gstreamer1.0-plugins-base libgtk-3-bin
#   libgts-bin javascript-common krb5-locales libtool opencv-data
#   libcoarrays-openmpi-dev libopenni-sensor-pointclouds0
#   | libopenni-sensor-primesense0 libpng-tools poppler-data
#   qttranslations5-l10n qt5-gtk-platformtheme geoclue-2.0 libraw1394-tools
#   va-driver-all | va-driver vdpau-driver-all | vdpau-driver
#   mesa-vulkan-drivers | vulkan-icd libwacom-bin file xauth netbase python3-gi
#   libpaper-utils python3-pil python3-click python3-bs4 python3-html5lib
#   python3-tk notification-daemon pyflakes3 python3-png networkd-dispatcher
#   libnss-systemd xterm | x-terminal-emulator


# bootstrap rosdep
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update

# setup colcon mixin and metadata
mkdir -p ${COLCON_SRC_DIR}
cd ${COLCON_WS_DIR}
. /opt/ros/${ROS2_DISTRO}/setup.bash
if [ ! -e ${HOME}/.colcon/mixin/default/asan.mixin ]
then
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
fi
colcon mixin update
if [ ! -e ${HOME}/.colcon/metadata/default/Gazebo.meta ]
then
    colcon metadata add default https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml
fi
colcon metadata update

echo
echo "Source the script /opt/ros/${ROS2_DISTRO}/setup.bash before use."
echo
