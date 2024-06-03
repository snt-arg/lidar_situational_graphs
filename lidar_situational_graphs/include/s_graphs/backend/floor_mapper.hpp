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

#ifndef FLOOR_MAPPER_HPP
#define FLOOR_MAPPER_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

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
#include <g2o/vertex_floor.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <iostream>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

/**
 * @brief
 */
class FloorMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor for the class FloorMapper
   *
   */
  FloorMapper();
  ~FloorMapper();

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param floors_vec
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void lookup_floors(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData room_data,
      std::unordered_map<int, s_graphs::Floors>& floors_vec,
      const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms);

 private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param floors_vec
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void factor_floor_node(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData room_data,
      std::unordered_map<int, s_graphs::Floors>& floors_vec,
      const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_node
   * @param room_data
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void update_floor_node(
      std::shared_ptr<GraphSLAM>& graph_slam,
      g2o::VertexFloor* floor_node,
      const situational_graphs_msgs::msg::RoomData room_data,
      const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms);

 private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_pose
   * @param floor_node
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void factor_floor_room_nodes(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const Eigen::Isometry3d& floor_pose,
      g2o::VertexFloor* floor_node,
      const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_node
   */
  void remove_floor_room_nodes(std::shared_ptr<GraphSLAM>& graph_slam,
                               g2o::VertexFloor* floor_node);
};

}  // namespace s_graphs

#endif  // FLOOR_MAPPER_HPP
