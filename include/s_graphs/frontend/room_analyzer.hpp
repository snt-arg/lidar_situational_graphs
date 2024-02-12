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

#ifndef ROOM_ANALYZER_HPP
#define ROOM_ANALYZER_HPP

#include <math.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <s_graphs/common/plane_utils.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/plane_data.hpp"
#include "s_graphs/msg/planes_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

struct room_analyzer_params {
  long int vertex_neigh_thres;
};

struct RoomInfo {
 public:
  const std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes;
  const std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;
};

struct RoomPlanes {
 public:
  s_graphs::msg::PlaneData& x_plane1;
  s_graphs::msg::PlaneData& x_plane2;
  s_graphs::msg::PlaneData& y_plane1;
  s_graphs::msg::PlaneData& y_plane2;
  bool found_x1_plane;
  bool found_x2_plane;
  bool found_y1_plane;
  bool found_y2_plane;
};

/**
 * @brief Class that provides tools for different analysis over open space
 * clusters to generate rooms
 */
class RoomAnalyzer {
 public:
  /**
   * @brief Constructor of class RoomAnalyzer
   *
   * @param private_nh
   * @param plane_utils_ptr
   */
  RoomAnalyzer(room_analyzer_params params);
  ~RoomAnalyzer();

  /**
   * @brief
   *
   * @param skeleton_graph_msg
   */
  void analyze_skeleton_graph(
      const visualization_msgs::msg::MarkerArray::SharedPtr&
          skeleton_graph_msg);

  /**
   * @brief
   *
   * @return Cloud clusters
   */
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extract_cloud_clusters() {
    return cloud_clusters;
  }

  /**
   * @brief
   *
   * @return Connected clusters marker array
   */
  visualization_msgs::msg::MarkerArray extract_marker_array_clusters() {
    return clusters_marker_array;
  }

  /**
   * @brief
   *
   * @param current_x_vert_planes
   * @param current_y_vert_planes
   * @param p_min
   * @param p_max
   * @param cloud_hull
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   * @param found_x1_plane
   * @param found_x2_plane
   * @param found_y1_plane
   * @param found_y2_plane
   * @return
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_room_planes(
      RoomInfo& room_info, RoomPlanes& room_planes, pcl::PointXY p_min,
      pcl::PointXY p_max);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param cloud_hull
   * @param area
   */
  void extract_convex_hull(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& area);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param p1
   * @param p2
   */
  void extract_cluster_endpoints(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      pcl::PointXY& p1, pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param p1
   * @param p2
   * @return Success or Failure
   */
  bool extract_centroid_location(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      const pcl::PointXY& p1, const pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param skeleton_cloud
   * @param room_center
   * @return Success or Failure
   */
  bool extract_centroid_location(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud,
      const geometry_msgs::msg::Point& room_center);

  /**
   * @brief Compute the distance between 2 points, p1 and p2.
   *
   * @param p1
   * @param p2
   * @return Distance between point p1 and point p2.
   */
  geometry_msgs::msg::Point extract_room_length(const pcl::PointXY& p1,
                                                const pcl::PointXY& p2);

  /**
   * @brief Compute the center of a room.
   *
   * @param plane_type
   * @param p1
   * @param p2
   * @param plane1
   * @param plane2
   * @param cluster_center
   * @return The center point of the room.
   */
  geometry_msgs::msg::Point extract_infinite_room_center(
      int plane_type, pcl::PointXY p1, pcl::PointXY p2,
      s_graphs::msg::PlaneData plane1, s_graphs::msg::PlaneData plane2,
      Eigen::Vector2d& cluster_center);

  /**
   * @brief
   *
   * @param current_x_vert_planes
   * @param current_y_vert_planes
   * @param cloud_cluster
   * @param cloud_hull
   * @param room_candidates_vec
   *
   * @return Success or Failure.
   */
  bool perform_room_segmentation(
      RoomInfo& room_info, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster,
      std::vector<s_graphs::msg::RoomData>& room_candidates_vec,
      const visualization_msgs::msg::MarkerArray& cloud_marker_array);

  /**
   * @brief
   *
   * @param cloud_hull
   */
  void downsample_cloud_data(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull);

 private:
  /**
   * @brief
   *
   * @param x_plane1_points
   * @param y_plane_points
   * @return Aligned or not aligned
   */
  bool is_x1_plane_aligned_w_y(
      const std::vector<geometry_msgs::msg::Vector3> x_plane1_points,
      const std::vector<geometry_msgs::msg::Vector3> y_plane_points);

  /**
   * @brief
   *
   * @param x_plane2_points
   * @param y_plane_point
   * @return Aligned or not aligned
   */
  bool is_x2_plane_aligned_w_y(
      const std::vector<geometry_msgs::msg::Vector3> x_plane2_points,
      const std::vector<geometry_msgs::msg::Vector3> y_plane_point);

  /**
   * @brief
   *
   * @param y_plane1_points
   * @param x_plane_points
   * @return Aligned or not aligned
   */
  bool is_y1_plane_aligned_w_x(
      const std::vector<geometry_msgs::msg::Vector3> y_plane1_points,
      const std::vector<geometry_msgs::msg::Vector3> x_plane_points);

  /**
   * @brief
   *
   * @param y_plane2_points
   * @param x_plane_points
   * @return Aligned or not aligned
   */
  bool is_y2_plane_aligned_w_x(
      const std::vector<geometry_msgs::msg::Vector3> y_plane2_points,
      const std::vector<geometry_msgs::msg::Vector3> x_plane_points);

 private:
  int vertex_neigh_thres;

  std::mutex skeleton_graph_mutex;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
  std::vector<std::pair<int, int>> subgraphs;
  visualization_msgs::msg::MarkerArray clusters_marker_array;

 private:
  /**
   * @brief
   *
   * @param start_point
   * @param end_point
   * @param plane
   * @return
   */
  std::vector<float> find_plane_points(const pcl::PointXY& start_point,
                                       const pcl::PointXY& end_point,
                                       const s_graphs::msg::PlaneData& plane);

  /**
   * @brief
   *
   * @param cloud_hull
   * @param plane
   * @param sub_cloud_cluster
   * @return
   */
  int find_plane_points(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull,
      const s_graphs::msg::PlaneData& plane,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sub_cloud_cluster);

  /**
   * @brief
   *
   * @param p1
   * @param p2
   * @return
   */
  pcl::PointXYZRGB compute_centroid(const pcl::PointXYZRGB& p1,
                                    const pcl::PointXYZRGB& p2);
};
}  // namespace s_graphs

#endif  // ROOM_ANALYZER_HPP
