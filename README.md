<div align="center">
 <h1>LiDAR S-Graphs</h1>

<a href="https://github.com/snt-arg/s_graphs/actions/workflows/foxy_build.yaml"><img src="https://github.com/snt-arg/s_graphs/actions/workflows/foxy_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/s_graphs/actions/workflows/humble_build.yaml"><img src="https://github.com/snt-arg/s_graphs/actions/workflows/humble_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/s_graphs/actions/workflows/iron_build.yaml"><img src="https://github.com/snt-arg/s_graphs/actions/workflows/iron_build.yaml/badge.svg" /></a>

</div>

**LiDAR Situational Graphs (S-Graphs)** is a ROS2 package for generating in real-time four-layered hierarchical factor graphs representing a scene graph using 3D LiDAR which includes **_Keyframes_** registring the robot poses, **_Walls_** which map wall planes, **_Rooms Layer_** constraining the wall planes using 4 wall-room or 2 wall-room factors, **_Floors_** constraining the rooms within a given floor level. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor). We have tested this package mostly with Velodyne (VLP16) sensors in structured indoor environments. This work is a fork of [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) which as previously in ROS1.

<p align="center">
  <a href="">
    <img src="./imgs/s_graphs+.gif" alt="Logo" width="80%">
  </a>
</p>

## 📜 Table of contents

- [📖 Published Papers](#published-papers)
- [⚙️ Installation](#installation)
  - [📦 Installation From Source](#installation-from-source)
    - [ROS1 Optional](#installation-for-ros1)
  - [🐳 Docker](#docker)
- [🚀 Examples on Datasets](#examples)
- [🛠️ Run S_Graphs On Your Data](#custom-data)
- [🤖 ROS Related](#ros-related)
  - [📥 Subscribed Topics](#subscribed-topics)
  - [📤 Published Topics](#published-topics)
  - [🔄 ROS Services](#ros-services)
  - [⚙️ ROS Parameters](#ros-parameters)
  - [🌐 Published TFs](#published-tfs)
- [🧪 Unit Tests](#unit-tests)

## 📖 Published Papers <a id="published-papers"></a>

<details >
    <summary><a href="https://arxiv.org/abs/2212.11770">S-Graphs+: Real-time Localization and Mapping leveraging Hierarchical Representations </a>
    </summary>

    @ARTICLE{10168233,
     author={Bavle, Hriday and Sanchez-Lopez, Jose Luis and Shaheer, Muhammad and Civera, Javier and Voos, Holger},
     journal={IEEE Robotics and Automation Letters},
     title={S-Graphs+: Real-Time Localization and Mapping Leveraging Hierarchical Representations},
     year={2023},
     volume={8},
     number={8},
     pages={4927-4934},
     doi={10.1109/LRA.2023.3290512}}

</details>
<details>
    <summary><a href="https://arxiv.org/abs/2202.12197">Situational Graphs for Robot Navigation in Structured Indoor Environments</a></summary>

      @ARTICLE{9826367,
        author={Bavle, Hriday and Sanchez-Lopez, Jose Luis and Shaheer, Muhammad and Civera, Javier and Voos, Holger},
        journal={IEEE Robotics and Automation Letters},
        title={Situational Graphs for Robot Navigation in Structured Indoor Environments},
        year={2022},
        volume={7},
        number={4},
        pages={9107-9114},
        doi={10.1109/LRA.2022.3189785}}

</details>

## ⚙️ Installation <a id="installation"></a>

<!-- TODO: When s-graphs is available in rosdistro add here the command to install -->

> [!NOTE]
> S-Graphs+ was only tested on Ubuntu 20.04, ROS2 Foxy, Humble Distros.
> We strongly recommend using [cyclone_dds](https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html) instead of the default fastdds.  

### 📦 Installation From Source <a id="installation-from-source"></a>

> [!IMPORTANT]
> Before proceeding, make sure you have `rosdep` installed. You can install it using `sudo apt-get install python3-rosdep`
> In addition, ssh keys are needed to be configured on you GitHub account. If you haven't
> yet configured ssh keys, follow this [tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

1. Update Rosdep:

```sh
rosdep init && rosdep update --include-eol-distros
```

2. Create a ROS2 workspace for S-Graphs

```bash
mkdir -p $HOME/workspaces/s_graphs_ros2_ws/src && cd $HOME/workspaces/s_graphs_ros2_ws/src && source /opt/ros/foxy/setup.bash
```

3. Clone the S-Graphs repository into the created workspace

```bash
git clone git@github.com:snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs
```

> [!IMPORTANT]
> If you have Nvidia GPU please install CUDA from this [link](https://developer.nvidia.com/cuda-11-8-0-download-archive). This code has only been tested with CUDA 11.8.
> If you dont have CUDA S-Graphs will use CPU only.

4. Install required dependencies

```bash
cd s_graphs && ./setup.sh
```

> [!NOTE]
> If you want to compile with debug traces (from backward_cpp) run:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

#### Optional ROS1 Install (Old Version of Room Segmentation) <a id="installation-for-ros1"></a>

<details>

> [!NOTE]
> This is an optional older version of room segmentation algorithm which requires ROS1 noetic. There is no hard dependecy on this package so you can easily ignore this step.

##### Download ROS Bridge <a id="download-ros-bridge"></a>

```bash
source /opt/ros/foxy/setup.bash && sudo apt install ros-foxy-ros1-bridge
```

##### Installation on ROS1

> [!IMPORTANT]
> Before following the instructions from below, ensure that you are in a fresh
> terminal, **without ROS2 sourced**.

1. Create a ROS1 workspace for S-Graphs

```bash
mkdir -p $HOME/workspaces/s_graphs_ros1_ws/src && cd $HOME/workspaces/s_graphs_ros1_ws/src && source /opt/ros/noetic/setup.bash
```

2. Clone the S-Graphs repository into the created workspace

```bash
git clone git@github.com:snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs
```

3. Install required dependencies using `vcstool`

```bash
cd s_graphs && vcs import --recursive ../ < .rosinstall_ros1
```

4. Install required ROS packages

```bash
cd ../../ && rosdep install --from-paths src --ignore-src -y -r
```

5. Install `pcl_ros`

```sh
sudo apt install ros-noetic-pcl-ros
```

6. Build workspace

> [!IMPORTANT]
> Make sure s_graphs_ros1_ws is built in Release otherwise the room extraction won't work properly.

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build
```

</details>

### 🐳 Docker <a id="docker"></a>

#### Build Docker Image

1. Create a ROS2 workspace for S-Graphs

```sh
mkdir -p $HOME/workspaces/s_graphs_ros2_ws/src && cd $HOME/workspaces/s_graphs_ros2_ws/src
```

2. Change directory to where Dockerfile is located in `s_graphs`

```sh
git clone git@github.com:snt-arg/lidar_s_graphs.git -b feature/ros2 s_graphs && cd $HOME/workspaces/s_graphs_ros2_ws/src/s_graphs/docker/foxy_noetic
```

3. Build image

> [!NOTE]
> In case you have a different ssh key name for your GitHub account, change `id_ed25519` oto yours.

```sh
docker build -t sntarg/s_graphs --ssh default=$HOME/.ssh/id_ed25519 .
```

## 🚀 Example on Datasets <a id="examples"></a>

> [!WARNING]
> For execution of the experiments we use [mprocs](https://github.com/pvolok/mprocs), which makes the process of launching different processes easier.

### Real Dataset

> [!IMPORTANT]
> Download real dataset using this [link](https://uniluxembourg-my.sharepoint.com/:u:/g/personal/hriday_bavle_uni_lu/ET2kNySZrzVBlveGgvSByeUBAvQk5wl05GMF0NwqbkL6ZA?e=hoaaOo) and store it in the folder `~/Downloads/real`, the below mprocs script will not work otherwise.

```bash
cd $HOME/workspaces/s_graphs_ros2_ws/src/s_graphs && mprocs --config .real_mprocs.yaml
```

### Virtual Dataset

> [!IMPORTANT]
> Download virtual dataset using this [link](https://uniluxembourg-my.sharepoint.com/:u:/g/personal/hriday_bavle_uni_lu/ETEfrz2n8qhKrXSJ712gNYgBtl5ra_9lUxZmsmyUa804ew?e=3XJOhG) and store it in the folder `~/Downloads/virtual`, the below mprocs script will not work otherwise.

```bash
cd $HOME/workspaces/s_graphs_ros2_ws/src/s_graphs && mprocs --config .virtual_mprocs.yaml
```

### Running S_Graphs with Docker

> [!NOTE]
> This tutorial assumes that you have followed the instructions to setup docker
> in section [🐳 Docker](#docker)

1. Create a container for the s_graphs image.

```bash
docker run -dit --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --network=host -e DISPLAY=$DISPLAY --name s_graphs_container sntarg/s_graphs
```

2. Download the dataset you desire from above to your **local machine**.

3. Move the rosbag inside docker container

```sh
docker cp ~/Downloads/real s_graphs_container:/root/Downloads/real # For real dataset
# OR
docker cp ~/Downloads/virtual s_graphs_container:/root/Downloads/virtual # For virtual dataset
```

4. Execute the container

```sh
docker exec -ti s_graphs_container bash
```

> [!IMPORTANT]
> If rviz2 doesnt open inside the docker, do `xhost +` in a terminal of your pc and then relaunch the mprocs command inside docker.

3. Run mprocs

```sh
mprocs_real # To run on a real robot or a real dataset
# OR
mprocs_virtual # To run on a simulation or virtual dataset
```

<!--
4. Visualization using Rviz (open another terminal to run this command)

> [!NOTE]
> The command below assumes you has ros2 foxy on your local machine

```sh
source /opt/ros/foxy/setup.bash && rviz2 -d $HOME/workspaces/s_graphs_ros2_ws/src/s_graphs/rviz/s_graphs_ros2.rviz
```
-->

> [!NOTE]
> Press reset on rviz2 once in a while when running S-Graphs to avoid freezing effect caused by rviz2 on foxy.

## 🛠️ Run S_Graphs On Your Data <a id="custom-data"></a>

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

> [!NOTE]
> If you want to visualize the tfs correctly from your odom source, you MUST provide a tf from the `odom` to `base_link` frame.

## 🤖 ROS Related <a id="ros-related"></a>

### 📥 Subscribed Topics <a id="subscribed-topics"></a>

#### `s_graphs` node

| Topic name         | Message Type                                                                                        | Description                              |
| ------------------ | --------------------------------------------------------------------------------------------------- | ---------------------------------------- |
| `/odom`            | [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)              | The odometry from the robot.             |
| `/filtered_points` | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | The filtered data from the LiDAR sensor. |

#### `room_segmentation` node

| Topic name                           | Message Type                                                                                                     | Description                                                                             |
| ------------------------------------ | ---------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| `/voxblox_skeletonizer/sparse_graph` | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) | Represents the free space where the robot can go to. Also known as free-space clusters. |
| `/s_graphs/map_planes`               | [s_graphs/PlanesData](https://github.com/snt-arg/s_graphs/blob/feature/ros2/msg/PlaneData.msg)                   | Planes seen by the current robot keyframe.                                              |

#### `floor_plan` node

| Topic name                 | Message Type                                                                                                     | Description                                      |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ |
| `/s_graphs/all_map_planes` | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) | All the planes that have been seen by the robot. |

### 📤 Published Topics <a id="published-topics"></a>

#### `s_graphs` node

| Topic name                      | Message Type                                                                                                     | Description                                                                   |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| `/s_graphs/markers`             | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) | These markers represent the different s_graphs layers.                        |
| `/s_graphs/odom2map`            | [geometry_msgs/TransformStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html)        | The estimated drift of the robot within its map frame (world).                |
| `/s_graphs/odom_pose_corrected` | [geometry_msgs/PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)           | The optimized/drift-free pose of the robot once odom2map is applied.          |
| `/s_graphs/odom_path_corrected` | [nav_msgs/Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)                                   | The optimized/drift-free pose path of the robot once the odom2map is applied. |
| `/s_graphs/map_points`          | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)              | The projected 3D points using the optimized robot pose.                       |
| `/s_graphs/map_planes`          | [s_graphs/PlanesData](https://github.com/snt-arg/s_graphs/blob/feature/ros2/msg/PlanesData.msg)                  | Planes seen by the current robot keyframe.                                    |
| `/s_graphs/all_map_planes`      | [s_graphs/PlanesData](https://github.com/snt-arg/s_graphs/blob/feature/ros2/msg/PlanesData.msg)                  | All the planes that have been seen by the robot.                              |

#### `room_segmentation` node

| Topic name                     | Message Type                                                                                  | Description                                                              |
| ------------------------------ | --------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| `/room_segmentation/room_data` | [s_graphs/RoomsData](https://github.com/snt-arg/s_graphs/blob/feature/ros2/msg/RoomsData.msg) | Contains all the necessary information about the rooms on a given floor. |

#### `floor_plan` node

| Topic name               | Message Type                                                                                 | Description                                              |
| ------------------------ | -------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| `/floor_plan/floor_data` | [s_graphs/RoomData](https://github.com/snt-arg/s_graphs/blob/feature/ros2/msg/RoomsData.msg) | Contains all the necessary information about each floor. |

### 🔄 ROS Services <a id="ros-services"></a>

| Topic name       | Message Type                                                                                  | Description                                                                                    |
| ---------------- | --------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `/s_graphs/dump` | [s_graphs/DumpGraph](https://github.com/snt-arg/s_graphs/blob/feature/ros2/srv/DumpGraph.srv) | Save all the internal data (point clouds, floor coeffs, odoms, and pose graph) to a directory. |

| Topic name           | Message Type                                                                              | Description                              |
| -------------------- | ----------------------------------------------------------------------------------------- | ---------------------------------------- |
| `/s_graphs/save_map` | [s_graphs/SaveMap](https://github.com/snt-arg/s_graphs/blob/feature/ros2/srv/SaveMap.srv) | Save the generated 3D map as a PCD file. |

### ⚙️ ROS Parameters <a id="ros-parameters"></a>

All the configurable parameters are listed in config folder as ros params.

### 🌐 Published TFs <a id="published-tfs"></a>

- `map2odom`: The transform published between the map frame and the odom frame after the corrections have been applied.

- The entire `tf_tree` for the virtual experiment can be seen in the figure below.

<p align="center">
  <a href="">
    <img src="./imgs/Tf-tree.png" alt="tf_tree" width="80%">
  </a>
</p>

## 🧪 Unit Tests <a id="unit-tests"></a>

Some unit tests are available. In case you want to add additional tests, run the following command:

```bash
colcon test --packages-select s_graphs --event-handler=console_direct+
```
