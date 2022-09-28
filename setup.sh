#!/bin/bash

# Assuming that we don't have a sperate repo for the s_graphs workspace, the script will only install the dependencies alongside the s_graph core.
# This can later be modified to make it the same as the stugalux_ws

vcs import --recursive ../ < .rosinstall
cd ../../ && rosdep install --from-paths src -ignore-src -y
catkin build

# - Sourcing the workspace
source devel/setup.bash


# - Here we could also download the datasets if they are online
