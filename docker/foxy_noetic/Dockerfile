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

# Authorize SSH Host (GitHub)
RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -H github.com >> /root/.ssh/known_hosts

#  Creating working spaces directory and clone s_graphs_ros2_ws inside it
RUN mkdir -p /root/workspaces/s_graphs_ros2_ws/src
WORKDIR /root/workspaces/s_graphs_ros2_ws/src
RUN --mount=type=ssh \
    git clone git@github.com:snt-arg/lidar_s_graphs.git -b feature/ros2

# Fetching all packages specified in the .rosinstall file and running build everything with colcon build.
WORKDIR /root/workspaces/s_graphs_ros2_ws/src/s_graphs
RUN --mount=type=ssh \
    vcs import --recursive ../ < .rosinstall_ros2
WORKDIR /root/workspaces/s_graphs_ros2_ws/
RUN sudo apt install python3-rosdep
RUN rosdep install --from-paths src -y --ignore-src -r
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
RUN --mount=type=ssh \
    git clone git@github.com:snt-arg/lidar_s_graphs.git -b feature/ros2
WORKDIR /root/workspaces/s_graphs_ros1_ws/src/s_graphs
RUN --mount=type=ssh \
    vcs import --recursive ../ < .rosinstall_ros1
WORKDIR /root/workspaces/s_graphs_ros1_ws
RUN rosdep install --from-paths src --ignore-src -y -r
RUN apt install --no-install-recommends -y \
    libtool \
    ros-noetic-pcl-ros \
    ros-foxy-ros1-bridge
RUN source /opt/ros/noetic/setup.bash && \
    catkin build


# Install mprocs
WORKDIR /root
RUN wget https://github.com/pvolok/mprocs/releases/download/v0.6.4/mprocs-0.6.4-linux64.tar.gz
RUN tar -xf mprocs* && \
    rm mprocs*.tar.gz && \
    mv mprocs /usr/local/bin

# alias for mprocs
RUN echo "alias mprocs_real='mprocs -c /root/workspaces/s_graphs_ros2_ws/src/s_graphs/.real_mprocs.yaml'" >> /root/.bashrc
RUN echo "alias mprocs_virtual='mprocs -c /root/workspaces/s_graphs_ros2_ws/src/s_graphs/.virtual_mprocs.yaml'" >> /root/.bashrc

# Removing SSH Host authorization (GitHub)
RUN rm -rf /root/.ssh/
