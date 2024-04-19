FROM ros:humble

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

#  Creating working spaces directory and clone s_graphs inside it
RUN mkdir -p /root/workspaces
WORKDIR /root/workspaces
RUN git clone https://github.com/snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs

# Fetching all packages specified in the .rosinstall file and running build everything with colcon build.
WORKDIR /root/workspaces/s_graphs
RUN git submodule update --init --recursive
WORKDIR /root/workspaces/s_graphs
RUN sudo apt install python3-rosdep
RUN rosdep install --from-paths . -y --ignore-src -r
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Install mprocs
WORKDIR /root
RUN wget https://github.com/pvolok/mprocs/releases/download/v0.6.4/mprocs-0.6.4-linux64.tar.gz
RUN tar -xf mprocs* && \
    rm mprocs*.tar.gz && \
    mv mprocs /usr/local/bin

# alias for mprocs
RUN echo "alias mprocs_real='mprocs -c /root/workspaces/s_graphs/.real_mprocs.yaml'" >> /root/.bashrc
RUN echo "alias mprocs_virtual='mprocs -c /root/workspaces/s_graphs/.virtual_mprocs.yaml'" >> /root/.bashrc