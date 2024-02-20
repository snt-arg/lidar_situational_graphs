# S-Graphs

**Situational graphs (S-Graphs)** is a ROS2 package for generating in real-time four-layered hierarchical factor graphs representing a scene graph including **_Keyframes_** registring the robot poses, **_Walls_** which map wall planes, **_Rooms Layer_** constraining the wall planes using 4 wall-room or 2 wall-room factors, **_Floors_** constraining the rooms within a given floor level. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor). We have tested this package mostly with Velodyne (VLP16) sensors in structured indoor environments. This work is a fork of [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) which as previously in ROS1.

<p align="center">
  <a href="">
    <img src="./imgs/s_graphs+.gif" alt="Logo" width="80%">
  </a>
</p>

## Table of contents

- [Published Papers](#published-papers)
- [About S-Graphs](#about-s-graphs)
  - [Architecture](#architecture)
- [Installation](#installation)
  - [Manual Installation](#manual-installation)
- [Example on Datasets](#example-on-datasets)
  - [Real Dataset](#real-dataset)
  - [Virtual Dataset](#virtual-dataset)
- [Unit Tests](#unit-tests-for-s-graphs)
- [Docker](#docker-note-docker-still-uses-the-ros1-older-version-of-s-graphs)
  - [Running Datasets Using Docker](#running-datasets-using-docker)
- [ROS2 Related](#ros2-related)
  - [Nodes](#nodes)
  - [Published TFs](#published-tfs)
  - [Services](#services)
  - [Parameters](#parameters)
- [Instructions To Use S-Graphs](#instructions-to-use-s-graphs)
- [License](#license)
- [Maintainers](#maintainers)

## Published Papers

1. [S-Graphs+: Real-time Localization and Mapping leveraging
Hierarchical Representations
   ](https://arxiv.org/abs/2212.11770)
   - **Citation**
     ```latex
      @ARTICLE{10168233,
        author={Bavle, Hriday and Sanchez-Lopez, Jose Luis and Shaheer, Muhammad and Civera, Javier and Voos, Holger},
        journal={IEEE Robotics and Automation Letters}, 
        title={S-Graphs+: Real-Time Localization and Mapping Leveraging Hierarchical Representations}, 
        year={2023},
        volume={8},
        number={8},
        pages={4927-4934},
        doi={10.1109/LRA.2023.3290512}}
     ```

2. [Situational Graphs for Robot Navigation in Structured Indoor Environments
   ](https://arxiv.org/abs/2202.12197)
   - **Citation**

     ```latex
      @ARTICLE{9826367,
        author={Bavle, Hriday and Sanchez-Lopez, Jose Luis and Shaheer, Muhammad and Civera, Javier and Voos, Holger},
        journal={IEEE Robotics and Automation Letters}, 
        title={Situational Graphs for Robot Navigation in Structured Indoor Environments}, 
        year={2022},
        volume={7},
        number={4},
        pages={9107-9114},
        doi={10.1109/LRA.2022.3189785}}
     ```

## About S-Graphs

### Architecture

<p align="center">
  <a href="">
    <img src="./imgs/system_architecture.png" alt="Architecture" width="80%">
  </a>
</p>

## Installation

> Note: S-Graphs+ is tested on Ubuntu 20.04 and ROS1 Noetic and ROS2 Foxy distro.

<!-- ### Automated Installation

1. Create a workspace for S-Graphs

```bash
mkdir -p $HOME/s_graphs_ros2_ws/src && cd $HOME/s_graphs_ros2_ws/src
```

2. Clone the S-Graphs repository into the created workspace (please setup your ssh keys first)

```bash
git clone git@github.com:snt-arg/s_graphs.git -b feature/ros2
```

3. Run the script setup.sh to install the required dependencies

```bash
cd s_graphs && ./setup.sh
``` -->

### Manual Installation

1. Create a ROS2 workspace for S-Graphs

```bash
mkdir -p $HOME/s_graphs_ros2_ws/src && cd $HOME/s_graphs_ros2_ws/src && source /opt/ros/foxy/setup.bash
```

2. Clone the S-Graphs repository into the created workspace

```bash
git clone https://github.com/snt-arg/s_graphs.git -b feature/ros2
```

3. Source your ROS2 evnironment and install the required dependencies using vcstool

```bash
cd s_graphs && vcs import --recursive ../ < .rosinstall_ros2
```

4. Install the required ROS2 packages

```bash
cd ../../ && rosdep install --from-paths src -y --ignore-src -r
```

5. Build workspace

```bash
colcon build --symlink-install
```

> If you want to compile with debug traces (from backward_cpp ) run

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
```

6. Source workspace

```bash
source install/setup.bash
```

#### Download ROS1 Bridge

```bash
source /opt/ros/foxy/setup.bash && sudo apt install ros-foxy-ros1-bridge
```

#### Setup ROS1 workspace for ROS1 related dependencies

1. Create a ros1 workspace for S-Graphs ros1 dependencies

```bash
mkdir -p $HOME/s_graphs_ros1_ws/src && cd $HOME/s_graphs_ros1_ws/src
```

2. Clone the S-Graphs repository into the created workspace

```bash
git clone https://github.com/snt-arg/s_graphs.git -b feature/ros2
```

3. Source your ROS1 evnironment and install the required dependencies using vcstool

```bash
cd s_graphs && vcs import --recursive ../ < .rosinstall_ros1
```

4. Install the required ROS packages

```bash
cd ../../ && rosdep install --from-paths src --ignore-src -y -r
```

5. Build workspace

```bash
catkin build
```

## Example on Datasets

> **Note:** For execution of the experiments we use [mprocs](https://github.com/pvolok/mprocs) command below. Download it using this [link](https://github.com/pvolok/mprocs)

### Real Dataset

> **Important:** Download real dataset using this [link](https://uniluxembourg-my.sharepoint.com/personal/hriday_bavle_uni_lu/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fhriday%5Fbavle%5Funi%5Flu%2FDocuments%2FARG%2FExperimentation%2FProjects%2FSTUGALUX%2Frosbags%2FStugalux%5FOetrange%2FReal%2Fstugalux%5Foetrange%5Ff2%5F3r%2Ebag&parent=%2Fpersonal%2Fhriday%5Fbavle%5Funi%5Flu%2FDocuments%2FARG%2FExperimentation%2FProjects%2FSTUGALUX%2Frosbags%2FStugalux%5FOetrange%2FReal&ga=1) and store it in the folder `~/Downloads/real`, the below mprocs script will not work otherwise.

```bash
cd $HOME/s_graphs_ros2_ws/src/s_graphs && mprocs --config .real_mprocs.yaml
```

### Virtual Dataset

> **Important:** Download virtual dataset using this [link](https://uniluxembourg-my.sharepoint.com/personal/hriday_bavle_uni_lu/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fhriday%5Fbavle%5Funi%5Flu%2FDocuments%2FARG%2FExperimentation%2FProjects%2FSTUGALUX%2Frosbags%2FStugalux%5FOetrange%2FSimulation%2Fstugalux%5Foetrange%5Ff2%5F3r%2Ebag&parent=%2Fpersonal%2Fhriday%5Fbavle%5Funi%5Flu%2FDocuments%2FARG%2FExperimentation%2FProjects%2FSTUGALUX%2Frosbags%2FStugalux%5FOetrange%2FSimulation&ga=1) and store it in the folder `~/Downloads/virtual`, the below mprocs script will not work otherwise.

```bash
cd $HOME/s_graphs_ros2_ws/src/s_graphs && mprocs --config .virtual_mprocs.yaml
```

## Docker (NOTE: Docker still uses the ROS1 (older) version of S-Graphs)

A docker image is provided with s_graphs. This image is all set and is just pull and play. Follow the instructions below in order to use s_graphs via docker.

1. Pull the docker image from DockerHub

```bash
docker pull sntarg/s_graphs:latest
```

2. Create a container for the s_graphs image.

```bash
docker run -dit --net host --name s_graphs_container sntarg/s_graphs
```

This command also incorporates the flags `d`, which makes the container run in the detached mode and `net`, which gives the container the access of the host interfaces.

3. Execute the container

```bash
docker exec -ti s_graphs_container bash
```

4. Source the s_graphs worspace

```bash
source devel/setup.bash
```

**Note:** Once the worspace is sourced once, it will no longer be required to resourced it again.

### Running Datasets using docker

In order to run datasets using docker, one just needs to use the command `roslaunch s_graphs s_graphs.launch use_free_space_graph:=true env:=virtual 2>/dev/null` inside docker.
The other 2 commands should be executed outside docker. Additionally, the `env` parameter should be changed accordingly to the type of dataset.

## Unit Tests for S-Graphs

Unit tests for some of the S-Graphs functions are in the folder tests. They can be executed using the following command

```bash
colcon test --packages-select s_graphs --event-handler=console_direct+
```

## ROS2 Related

### Nodes

> s_graphs is composed of **3** main nodes.

- **s_graphs_node**

  - Subscribed Topics

    - `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
      - The odometry from the robot.
    - `/filtered_points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))
      - The filtered data from the LiDAR sensor.

  - Published Topics

    - `/s_graphs/markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
      - These markers represent the different s_graphs layers.
    - `/s_graphs/odom2map` ([geometry_msgs/TransformStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html))
      - The estimated drift of the robot within its map frame (world).
    - `/s_graphs/odom_pose_corrected` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
      - The optimized/drift-free pose of the robot once odom2map is applied.
    - `/s_graphs/odom_path_corrected` ([nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
      - The optimized/drift-free pose path of the robot once the odom2map is applied.

    - `/s_graphs/map_points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))
      - The projected 3D points using the optimized robot pose.
    - `/s_graphs/map_planes` (s_graphs/PlanesData)
      - Planes seen by the current robot keyframe.
    - `/s_graphs/all_map_planes` (s_graphs/PlanesData)
      - All the planes that have been seen by the robot.

- **room_segmentation_node**

  - Subscribed Topics

    - `/voxblox_skeletonizer/sparse_graph` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
      - Represents the free space where the robot can go to. This is also knonw as free-space clusters.
    - `/s_graphs/map_planes` (s_graphs/PlanesData)
      - Planes seen by the current robot keyframe.

  - Published Topics

    - `/room_segmentation/room_data` (s_graphs/RoomsData)
      - Contains all the necessary information about the rooms on a given floor.

- **floor_plan_node**

  - Subscribed Topics

    - `/s_graphs/all_map_planes` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
      - All the planes that have been seen by the robot.

  - Published Topics

    - `/floor_plan/floor_data` (s_graphs/RoomData):
      - Constains all the necessary information about each floor.

### Services

- `/s_graphs/dump` (s_graphs/DumpGraph)

  - Save all the internal data (point clouds, floor coeffs, odoms, and pose graph) to a directory.

- `/s_graphs/save_map` (s_graphs/SaveMap)
  - Save the generated 3D map as a PCD file.

### Parameters

All the configurable parameters are listed in config folder as ros params.

### Published TFs

- `map2odom`: The transform published between the map frame and the odom frame after the corrections have been applied.

- The entire `tf_tree` for the virtual experiment can be seen in the figure below.

<p align="center">
  <a href="">
    <img src="./imgs/Tf-tree.png" alt="tf_tree" width="80%">
  </a>
</p>

## Instructions To Use S-Graphs on Custom Dataset

1. Define the transformation between your sensors (LIDAR, IMU, GPS) and base_link of your system using static_transform_publisher (see [line](https://github.com/snt-arg/s_graphs/blob/c0489660552cb3a2fc8ac0bef17998ee5fb6e15a/launch/s_graphs_launch.py#L118), s_graphs_launch.py). All the sensor data will be transformed into the common `base_link` frame, and then fed to the SLAM algorithm. Note: `base_link` frame in virtual dataset is set to `base_footprint` and in real dataset is set to `body`

2. Remap the point cloud topic in s_graphs_launch.py of **s_graphs_prefiltering_node** and **s_graphs_node**. Like:

```python
    remappings=[
            ("velodyne_points", "/rs_lidar/points"),
            ("imu/data", "/platform/imu/data"),
        ],
  ...
```

3. If you have an odometry source convert it to base ENU frame, then set the arg `compute_odom` to `false` in `s_graphs_ros2_launch.py` and then remap odom topic in **s_graphs_node** like

```python
     remappings=[
            ("velodyne_points", "/platform/velodyne_points"),
            ("/odom", "/husky/odometry"),
        ],
  ...
```

> Note: If you want to visualize the tfs correctly from your odom source, you MUST provide a tf from the `odom` to `base_link` frame.

## License

This package is released under the **BSD-2-Clause** License.

Note that the cholmod solver in g2o is licensed under GPL. You may need to build g2o without cholmod dependency to avoid the GPL.

## Maintainers

- <ins>**Hriday Bavle**</ins>
  - **Email:** hriday.bavle@uni.lu
  - **Website:** <https://www.hriday.bavle.com/>
- <ins>**Muhammad Shaheer**</ins>
  - **Email:** muhamad.shaheer@uni.lu
- <ins>**Pedro Soares**</ins>
  - **Email:** pedro.soares@uni.lu
