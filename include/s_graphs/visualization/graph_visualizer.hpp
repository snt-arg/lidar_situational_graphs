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

#ifndef GRAPH_VISUALIZER_HPP
#define GRAPH_VISUALIZER_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/format.hpp>
#include <cmath>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <iostream>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/plane_mapper.hpp>
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/common/door_ways.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <string>

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/point_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.h"

namespace s_graphs {

/**
 * @brief
 */
class GraphVisualizer {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor for class GraphVisualizer.
   *
   * @param node
   */
  GraphVisualizer(const rclcpp::Node::SharedPtr node);
  ~GraphVisualizer();

 public:
  /**
   * @brief Creates a marker array
   *
   * @param stamp
   * @param local_graph
   * @param x_plane_snapshot
   * @param y_plane_snapshot
   * @param loop_detector_radius
   * @param keyframes
   * @param floors_vec
   * @return A MarkerArray message.
   */
  visualization_msgs::msg::MarkerArray create_marker_array(
      const rclcpp::Time& stamp,
      const g2o::SparseOptimizer* local_graph,
      const std::vector<VerticalPlanes>& x_plane_snapshot,
      const std::vector<VerticalPlanes>& y_plane_snapshot,
      const std::vector<HorizontalPlanes>& hort_plane_snapshot,
      std::vector<InfiniteRooms> x_infinite_room_snapshot,
      std::vector<InfiniteRooms> y_infinite_room_snapshot,
      std::vector<Rooms> room_snapshot,
      double loop_detector_radius,
      std::vector<KeyFrame::Ptr> keyframes,
      std::vector<Floors> floors_vec);

  visualization_msgs::msg::MarkerArray create_prior_marker_array(
      const rclcpp::Time& stamp,
      const g2o::SparseOptimizer* local_graph,
      std::vector<VerticalPlanes>& x_vert_planes_prior,
      std::vector<VerticalPlanes>& y_vert_planes_prior,
      std::vector<Rooms> rooms_vec_prior,
      std::vector<Rooms> rooms_vec,
      bool got_trans_prior2map_,
      const std::vector<DoorWays> doorways_vec_prio,
      std::vector<VerticalPlanes>& x_vert_planes,
      std::vector<VerticalPlanes>& y_vert_planes);

  Eigen::Isometry3d compute_plane_pose(const VerticalPlanes& plane,
                                       pcl::PointXYZRGBNormal& p_min,
                                       pcl::PointXYZRGBNormal& p_max);

 private:
  std::string map_frame_id;
  double color_r, color_g, color_b;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::Node* node_ptr_;
};

}  // namespace s_graphs

#endif  // GRAPH_VISUALIZER_HPP
