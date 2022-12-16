FROM ros:noetic

# - ssh keys to get the s_graphs repo (only temporary)
ARG ssh_prv_key
ARG ssh_pub_key

RUN apt-get update && apt-get install -y --no-install-recommends libtool \
    wget nano vim build-essential libomp-dev clang lld git\
    ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs \
    ros-noetic-rviz ros-noetic-tf-conversions ros-noetic-libg2o \
    python3 python3-pip python3-vcstool git \
    openssh-server libmysqlclient-dev

RUN pip3 install -U catkin_tools

# Authorize SSH Host (only temporary)
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan github.com > /root/.ssh/known_hosts

# Add the ssht keys and set permissions (only temporary)
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub

# - Creating S-Graphs working space
RUN mkdir -p /root/s_graphs_ws/src
WORKDIR /root/s_graphs_ws/src
RUN git clone git@github.com:snt-arg/s_graphs.git && cd s_graphs && git checkout v2.1.0 -b v2.1.0-stable-release

# - Installing S-Graphs dependencies
WORKDIR /root/s_graphs_ws/src/s_graphs
RUN vcs import --recursive ../ < .rosinstall
WORKDIR /root/s_graphs_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# - Updating rosdep
RUN rosdep update

# - Catkin build
RUN catkin config --extend /opt/ros/noetic/ && catkin build -j10
# RUN catkin config --install

# - Removing cpp source files from s_graphs
RUN rm src/s_graphs/src/s_graphs/*.cpp && rm -rf src/s_graphs/apps

# Remove SSH keys
RUN rm -rf /root/.ssh/
