<div align="center">
 <h1>LiDAR S-Graphs</h1>
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/foxy_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/foxy_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/humble_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/humble_build.yaml/badge.svg" /></a>
<a href="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/iron_build.yaml"><img src="https://github.com/snt-arg/lidar_situational_graphs/actions/workflows/iron_build.yaml/badge.svg" /></a>
</div>

**LiDAR Situational Graphs (S-Graphs)** is a ROS2 package for generating in real-time four-layered hierarchical factor graphs for single or multi-floor scenes. It reepresents a scene graph using 3D LiDAR which includes **_Keyframes_** registring the robot poses, **_Walls_** which map wall planes, **_Rooms Layer_** constraining the wall planes using 4 wall-room or 2 wall-room factors, **_Floors_** constraining the rooms within a given floor level. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor). We have tested this package mostly with Ouster OS-1 and Velodyne (VLP16) sensors in structured indoor environments. 

<p align="center">
  <a href="https://youtu.be/wg0_VSo4iE8" target="_blank">
    <img src="./docs/imgs/main_image.png" alt="Video Thumbnail" width="80%">
  </a>
</p>

## Documentation

For installation refer to the documentation available [here](https://snt-arg.github.io/lidar_situational_graphs)
