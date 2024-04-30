FROM ros:foxy

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    ssh \
    vim \
    python3-pip \
    python3-vcstool \
    python3-catkin-tools \
    wget \
    git

# Add the requirements.txt file and install the libraries
# ADD requirements.txt requirements.txt
# RUN pip3 install -r requirements.txt

# Install ros2 packages
RUN apt install --no-install-recommends -y \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-tf-transformations \
    ros-foxy-xacro \
    ros-foxy-depth-image-proc \
    ros-foxy-velodyne \
    ros-foxy-teleop-twist-keyboard

#  Creating working spaces directory and clone s_graphs inside it
RUN mkdir -p /root/workspaces
WORKDIR /root/workspaces
RUN git clone https://github.com/snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs

# Fetching all packages specified in the gitsubmodules and running build everything with colcon build.
WORKDIR /root/workspaces/s_graphs
RUN git submodule update --init --recursive
WORKDIR /root/workspaces/s_graphs
RUN sudo apt install python3-rosdep
RUN rosdep install --from-paths . -y --ignore-src -r
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# Install ROS1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install --no-install-recommends -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update
RUN apt install --no-install-recommends -y \
    ros-noetic-desktop \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential
RUN rosdep update --include-eol-distros

# Setup ROS1 workspace for ROS1 related dependencies of s_graphs
RUN mkdir -p /root/workspaces/s_graphs_ros1_ws/src
WORKDIR /root/workspaces/s_graphs_ros1_ws/src
RUN git clone https://github.com/snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs
WORKDIR /root/workspaces/s_graphs_ros1_ws/src/s_graphs
RUN vcs import --recursive ../ < .rosinstall_ros1
WORKDIR /root/workspaces/s_graphs_ros1_ws
RUN rosdep install --from-paths src --ignore-src -y -r
RUN apt install --no-install-recommends -y \
    libtool \
    ros-noetic-pcl-ros \
    ros-foxy-ros1-bridge
RUN source /opt/ros/noetic/setup.bash && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \ 
    catkin build

# Install mprocs
WORKDIR /root
RUN wget https://github.com/pvolok/mprocs/releases/download/v0.6.4/mprocs-0.6.4-linux64.tar.gz
RUN tar -xf mprocs* && \
    rm mprocs*.tar.gz && \
    mv mprocs /usr/local/bin

# alias for mprocs
RUN echo "alias mprocs_real='mprocs -c /root/workspaces/s_graphs/.real_mprocs.yaml'" >> /root/.bashrc
RUN echo "alias mprocs_virtual='mprocs -c /root/workspaces/s_graphs/.virtual_mprocs.yaml'" >> /root/.bashrc