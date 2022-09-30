#!/bin/bash

# - Installing catkin tools
sudo pip3 install -U catkin_tools

# - Installing vcstool
sudo apt install python3-vcstool

# - Importing all dependencies
vcs import --recursive ../ < .rosinstall
cd ../../ && rosdep install --from-paths src --ignore-src -r -y
catkin build

# - Sourcing the workspace
source devel/setup.bash
