/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

// SPDX-License-Identifier: BSD-2-Clause

#ifndef WALL_MAPPER_HPP
#define WALL_MAPPER_HPP

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/format.hpp>
#include <cmath>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/plane_data.hpp"
#include "s_graphs/msg/planes_data.hpp"
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

class WallMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Contructor of class WallMapper
   *
   * @param private_nh
   */
  WallMapper(const rclcpp::Node::SharedPtr node);
  ~WallMapper();

 public:
  /**
   * @brief
   *
   * @param covisibility_graph
   * @param x_planes_msg
   * @param x_wall_pose
   * @param y_planes_msg
   * @param y_wall_pose
   * @param x_vert_planes
   * @param y_vert_planes
   * @return
   */

 public:
  void factor_wall(std::shared_ptr<GraphSLAM>& covisibility_graph,
                   std::vector<s_graphs::msg::PlaneData> x_planes_msg,
                   Eigen::Vector3d x_wall_pose,
                   std::vector<s_graphs::msg::PlaneData> y_planes_msg,
                   Eigen::Vector3d y_wall_pose,
                   std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                   std::unordered_map<int, VerticalPlanes>& y_vert_planes);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param wall_pose
   * @param plane1
   * @param plane2
   * @return
   */
  void add_wall_node_and_edge(std::shared_ptr<GraphSLAM>& covisibility_graph,
                              Eigen::Vector3d wall_pose,
                              VerticalPlanes& plane1,
                              VerticalPlanes& plane2);

 private:
  bool use_point_to_plane;

 private:
  std::unique_ptr<PlaneUtils> plane_utils;
  rclcpp::Node::SharedPtr node_obj;
};

}  // namespace s_graphs

#endif  // WALL_MAPPER_HPP
