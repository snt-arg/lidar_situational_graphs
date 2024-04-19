#!/bin/bash

# - Installing cpp dependencies
sudo apt update

#import all repos
vcs import --recursive < .rosinstall_ros2

#rosdep install
rosdep install --from-paths . -y --ignore-src -r