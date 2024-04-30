#!/bin/bash

sudo apt update
DEBIAN_FRONTEND=noninteractive sudo apt install -y libceres-dev python3-pip

#rosdep init
rosdep update --include-eol-distros

#import all repos
git submodule update --init --recursive

#rosdep install
DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install