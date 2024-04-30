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



#ifndef GRAPH_VISUALIZER_HPP
#define GRAPH_VISUALIZER_HPP

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
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <string>

#include "geometry_msgs/msg/point.h"
#include "geometry_msgs/msg/point_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "rviz_visual_tools/rviz_visual_tools.hpp"
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
      const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
      const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
      const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot,
      std::unordered_map<int, InfiniteRooms> x_infinite_room_snapshot,
      std::unordered_map<int, InfiniteRooms> y_infinite_room_snapshot,
      std::unordered_map<int, Rooms> room_snapshot,
      double loop_detector_radius,
      std::vector<KeyFrame::Ptr> keyframes,
      std::unordered_map<int, Floors> floors_vec);

  /**
   * @brief Create a compressed graph object
   *
   * @param stamp
   * @param global_optimization
   * @param room_optimization
   * @param compressed_graph
   * @param x_plane_snapshot
   * @param y_plane_snapshot
   * @param hort_plane_snapshot
   */
  void create_compressed_graph(
      const rclcpp::Time& stamp,
      bool global_optimization,
      bool room_optimization,
      const g2o::SparseOptimizer* compressed_graph,
      const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
      const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
      const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot);

 private:
  /**
   * @brief
   *
   * @param current_plane_id
   * @param plane_snapshot
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d compute_vert_plane_centroid(
      const int current_plane_id,
      const std::unordered_map<int, VerticalPlanes>& plane_snapshot);

  /**
   * @brief
   *
   * @param current_plane_id
   * @param plane_snapshot
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d compute_hort_plane_centroid(
      const int current_plane_id,
      const std::unordered_map<int, HorizontalPlanes>& plane_snapshot);

  /**
   * @brief Create a prior marker array object
   *
   * @param stamp
   * @param local_graph
   * @param x_vert_planes_prior
   * @param y_vert_planes_prior
   * @param rooms_vec_prior
   * @param rooms_vec
   * @param got_trans_prior2map_
   * @param doorways_vec_prio
   * @param x_vert_planes
   * @param y_vert_planes
   * @return * visualization_msgs::msg::MarkerArray
   */
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

  /**
   * @brief
   *
   * @param plane
   * @param p_min
   * @param p_max
   * @return * Eigen::Isometry3d
   */
  Eigen::Isometry3d compute_plane_pose(const VerticalPlanes& plane,
                                       pcl::PointXYZRGBNormal& p_min,
                                       pcl::PointXYZRGBNormal& p_max);

  /**
   * @brief
   *
   * @param room_p1
   * @param cloud_seg_map
   * @return * geometry_msgs::msg::Point
   */
  geometry_msgs::msg::Point compute_plane_point(
      geometry_msgs::msg::Point room_p1,
      const pcl::PointCloud<PointNormal>::Ptr cloud_seg_map);

  /**
   * @brief
   *
   * @param room_p1
   * @return geometry_msgs::msg::Point
   */
  geometry_msgs::msg::Point compute_room_point(geometry_msgs::msg::Point room_p1);

 private:
  std::string map_frame_id;
  double color_r, color_g, color_b;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::Node* node_ptr_;
  std::string keyframes_layer_id;
  std::string walls_layer_id;
  std::string rooms_layer_id;
  std::string floors_layer_id;
  rviz_visual_tools::RvizVisualToolsPtr keyframe_node_visual_tools,
      keyframe_edge_visual_tools, plane_node_visual_tools, plane_edge_visual_tools,
      room_node_visual_tools, floor_node_visual_tools, floor_edge_visual_tools;
};

}  // namespace s_graphs

#endif  // GRAPH_VISUALIZER_HPP
