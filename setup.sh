#!/bin/bash

# - Installing cpp dependencies
sudo apt install python3-vcstool
sudo apt install libceres-dev

#install python dependencies
pip3 install torch==2.0.1+cu118 -f https://download.pytorch.org/whl/torch_stable.html
pip3 install torch-geometric==2.3.1
pip3 install torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
pip3 install torch-sparse==0.6.17+pt20cu118 -f https://data.pyg.org/whl/torch-2.0.1+cu118.html
pip3 install pyg-lib -f https://data.pyg.org/whl/torch-2.0.1+cu118.html
pip install protobuf==3.20.*

#rosdep init
rosdep init && rosdep update --include-eol-distros

#import all repos
vcs import --recursive ../ < .rosinstall_ros2

#install python requirements
pip3 install -r requirements.txt

#rosdep install
cd ../../ && rosdep install --from-paths src -y --ignore-src -r

# - Importing all dependencies
colcon build --symlink-install 

# - Sourcing the workspace
source install/setup.bash