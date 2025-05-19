FROM osrf/ros:humble-desktop-full AS base

ARG COLCON_JOBS=4

ENV DEBIAN_FRONTEND=noninteractive
ENV MAKEFLAGS="-j $COLCON_JOBS"

# Install necessary and useful packages
RUN apt update && \
    apt install --no-install-recommends -y \
    vim python3-pip python3-vcstool wget git \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp 

FROM base AS optional

# Install mprocs
WORKDIR /opt

# Install Cargo 
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y

# Install Mprocs
RUN /root/.cargo/bin/cargo install mprocs

FROM base AS sgraphs_prerequisites

RUN pip3 install scikit-learn torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2

# Clone lidar_situational_graphs
COPY . /workspace/src/s_graphs
# RUN git clone https://github.com/snt-arg/lidar_situational_graphs s_graphs
WORKDIR /workspace/src/s_graphs
RUN vcs import . < .rosinstall_ros2

FROM sgraphs_prerequisites AS sgraphs_build

# Build lidar_situational_graphs
WORKDIR /workspace
RUN rosdep update && \
    rosdep install --from-paths . -y --ignore-src -r
RUN . /opt/ros/humble/setup.sh && colcon build 


# Copy mprocs bin
COPY --from=optional /root/.cargo/bin/mprocs /usr/bin

# alias for mprocs
RUN echo "alias mprocs_real='mprocs -c /workspace/src/s_graphs/.docker_real_mprocs.yaml'" >> ~/.bashrc
RUN echo "alias mprocs_virtual='mprocs -c /workspace/src/s_graphs/.docker_virtual_mprocs.yaml'" >> ~/.bashrc

# Build Entrypoint
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /entrypoint.sh \
    && echo "source /workspace/install/setup.bash" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
