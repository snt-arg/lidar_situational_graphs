# LiDAR S-Graphs {align=center}

<div align="center">
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/foxy_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/foxy_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/humble_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/humble_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/iron_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/iron_build.yaml/badge.svg" /></a>
</div>
---

**LiDAR Situational Graphs (S-Graphs)** is a ROS2 package for generating in real-time four-layered hierarchical factor graphs for single or multi-floor scenes. It reepresents a scene graph using 3D LiDAR which includes **_Keyframes_** registring the robot poses, **_Walls_** which map wall planes, **_Rooms Layer_** constraining the wall planes using 4 wall-room or 2 wall-room factors, **_Floors_** constraining the rooms within a given floor level. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor). We have tested this package mostly with Ouster OS-1 and Velodyne (VLP16) sensors in structured indoor environments. This work is a fork of [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) which as previously in ROS1.

<div align="center">
  <video width="600" autoplay loop muted controls>
    <source src="./imgs/main_building.mp4" type="video/mp4">
    Your browser does not support the video tag.
  </video>
</div>

**Additional video:** [Dataset Comparison](https://youtu.be/vOH8X7XnQms?si=QtxYDyQhrSazDjqB)


