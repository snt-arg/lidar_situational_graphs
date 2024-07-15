#!/bin/bash

sudo apt update
DEBIAN_FRONTEND=noninteractive sudo apt install -y libceres-dev python3-pip

#rosdep init
rosdep update --include-eol-distros

#install vcs-tools
DEBIAN_FRONTEND=noninteractive sudo apt install python3-vcstool

#import all repos
vcs import . < .rosinstall_ros2

#rosdep install
DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install