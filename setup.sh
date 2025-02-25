#!/bin/bash

# - Installing cpp dependencies
sudo apt update
sudo apt install -y python3-vcstool
sudo apt install -y libceres-dev
sudo apt install -y python3-pip

#install python dependencies
pip3 install torch==2.0.1+cu118 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install torch-geometric==2.3.1
pip3 install torch-sparse==0.6.17+pt20cu118 -f https://data.pyg.org/whl/torch-2.0.1+cu118.html
pip3 install pyg-lib -f https://data.pyg.org/whl/torch-2.0.1+cu118.html
pip install protobuf==3.20.*

#rosdep init
rosdep update --include-eol-distros

#import all repos
vcs import . < .rosinstall_ros2

#install python requirements
pip3 install -r requirements.txt

#rosdep install
rosdep install --from-paths . -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install --cmake-args -DUSE_RGB_CLOUD=ON

# - Sourcing the workspace
source install/setup.bash