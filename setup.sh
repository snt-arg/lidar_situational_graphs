#!/bin/bash

# - Importing all dependencies
vcs import --recursive ../ < .rosinstall
cd ../../ && rosdep install --from-paths src -ignore-src -y
catkin build

# - Sourcing the workspace
source devel/setup.bash
