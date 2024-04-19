#!/bin/bash

sudo apt update
DEBIAN_FRONTEND=noninteractive sudo apt install -y libceres-dev

#rosdep init
rosdep update --include-eol-distros

#import all repos
git submodule update --init --recursive

#install python requirements
pip3 install -r requirements.txt

#rosdep install
DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install