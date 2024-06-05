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

#ifndef GRAPH_PUBLISHER_HPP
#define GRAPH_PUBLISHER_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/format.hpp>
#include <cmath>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_wall_two_planes.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <g2o/vertex_wall.hpp>
#include <iostream>
#include <s_graphs/backend/floor_mapper.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/keyframe_mapper.hpp>
#include <s_graphs/backend/plane_mapper.hpp>
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/map_cloud_generator.hpp>
#include <s_graphs/common/nmea_sentence_parser.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/loop_detector.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/common/room_utils.hpp"
#include "s_graphs/common/ros_utils.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "situational_graphs_reasoning_msgs/msg/attribute.hpp"
#include "situational_graphs_reasoning_msgs/msg/edge.hpp"
#include "situational_graphs_reasoning_msgs/msg/graph.hpp"
#include "situational_graphs_reasoning_msgs/msg/graph_keyframes.hpp"
#include "situational_graphs_reasoning_msgs/msg/node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class GraphPublisher {
 public:
  GraphPublisher();
  ~GraphPublisher();

 public:
  /**
   * @brief
   *
   * @param local_graph
   * @param graph_type
   * @param x_vert_planes_prior
   * @param y_vert_planes_prior
   * @param rooms_vec_prior
   * @param x_vert_planes
   * @param y_vert_planes
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @return situational_graphs_reasoning_msgs::msg::Graph
   */
  situational_graphs_reasoning_msgs::msg::Graph publish_graph(
      const g2o::SparseOptimizer* local_graph,
      std::string graph_type,
      const std::vector<s_graphs::VerticalPlanes>& x_vert_planes_prior,
      const std::vector<s_graphs::VerticalPlanes>& y_vert_planes_prior,
      const std::vector<s_graphs::Rooms>& rooms_vec_prior,
      const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param local_graph
   * @param keyframes
   * @return situational_graphs_reasoning_msgs::msg::GraphKeyframes
   */
  situational_graphs_reasoning_msgs::msg::GraphKeyframes publish_graph_keyframes(
      const g2o::SparseOptimizer* local_graph,
      const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes);

  /**
   * @brief
   *
   * @param local_graph
   * @param keyframes
   * @return situational_graphs_reasoning_msgs::msg::GraphKeyframes
   */
  situational_graphs_reasoning_msgs::msg::GraphKeyframes publish_graph_keyframes(
      const g2o::SparseOptimizer* local_graph,
      const std::vector<s_graphs::KeyFrame::Ptr>& keyframes);

  /**
   * @brief
   *
   * @param local_graph
   * @param rooms_vec
   */
  void publish_room_keyframes(const g2o::SparseOptimizer* local_graph,
                              const std::vector<s_graphs::Rooms>& rooms_vec);

 private:
};

#endif  // GRAPH_PUBLISHER_HPP
