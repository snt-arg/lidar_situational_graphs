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

## 📖 Published Papers <a id="published-papers"></a>


<details >
    <summary><a href="https://arxiv.org/abs/2502.18044">S-Graphs 2.0 -- A Hierarchical-Semantic Optimization and Loop Closure for SLAM </a>
    </summary>

    @misc{bavle2025sgraphs20hierarchicalsemantic,
      title={S-Graphs 2.0 -- A Hierarchical-Semantic Optimization and Loop Closure for SLAM}, 
      author={Hriday Bavle and Jose Luis Sanchez-Lopez and Muhammad Shaheer and Javier Civera and Holger Voos},
      year={2025},
      eprint={2502.18044},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2502.18044}, 
}

</details>

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
