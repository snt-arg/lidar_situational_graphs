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

#ifndef PLANE_MAPPER_HPP
#define PLANE_MAPPER_HPP

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

/**
 * @brief
 */
class PlaneMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Contructor of class PlaneMapper
   *
   * @param private_nh
   */
  PlaneMapper(const rclcpp::Node::SharedPtr node);
  ~PlaneMapper();

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param keyframe
   * @param extracted_cloud_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void map_extracted_planes(
      std::shared_ptr<GraphSLAM>& graph_slam,
      KeyFrame::Ptr keyframe,
      const std::vector<pcl::PointCloud<PointNormal>::Ptr>& extracted_cloud_vec,
      std::vector<VerticalPlanes>& x_vert_planes,
      std::vector<VerticalPlanes>& y_vert_planes,
      std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param keyframe
   * @param det_plane_body_frame
   * @return
   */
  g2o::Plane3D convert_plane_to_map_frame(const KeyFrame::Ptr& keyframe,
                                          const g2o::Plane3D& det_plane_body_frame);

  /**
   * @brief
   *
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void convert_plane_points_to_map(std::vector<VerticalPlanes>& x_vert_planes,
                                   std::vector<VerticalPlanes>& y_vert_planes,
                                   std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param plane_type
   * @param keyframe
   * @param det_plane
   * @param cloud_seg_body
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  std::pair<int, int> associate_plane(
      const int& plane_type,
      const KeyFrame::Ptr& keyframe,
      const g2o::Plane3D& det_plane,
      const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
      const std::vector<VerticalPlanes>& x_vert_planes,
      const std::vector<VerticalPlanes>& y_vert_planes,
      const std::vector<HorizontalPlanes>& hort_planes);

 private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param keyframe
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int add_planes_to_graph(std::shared_ptr<GraphSLAM>& graph_slam,
                          KeyFrame::Ptr& keyframe,
                          const g2o::Plane3D& det_plane_body_frame,
                          std::vector<VerticalPlanes>& x_vert_planes,
                          std::vector<VerticalPlanes>& y_vert_planes,
                          std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int sort_planes(std::shared_ptr<GraphSLAM>& graph_slam,
                  const int& plane_type,
                  KeyFrame::Ptr& keyframe,
                  const g2o::Plane3D& det_plane_map_frame,
                  const g2o::Plane3D& det_plane_body_frame,
                  std::vector<VerticalPlanes>& x_vert_planes,
                  std::vector<VerticalPlanes>& y_vert_planes,
                  std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int factor_planes(std::shared_ptr<GraphSLAM>& graph_slam,
                    const int& plane_type,
                    KeyFrame::Ptr& keyframe,
                    const g2o::Plane3D& det_plane_map_frame,
                    const g2o::Plane3D& det_plane_body_frame,
                    std::vector<VerticalPlanes>& x_vert_planes,
                    std::vector<VerticalPlanes>& y_vert_planes,
                    std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_id
   * @param keyframe
   * @param det_plane_map_frame
   * @param found_infinite_room
   * @param found_room
   * @param plane_id_pair
   */
  void retrieve_plane_properties(const int& plane_type,
                                 const int& plane_id,
                                 const KeyFrame::Ptr& keyframe,
                                 const g2o::Plane3D& det_plane_map_frame,
                                 bool& found_infinite_room,
                                 bool& found_room,
                                 plane_data_list& plane_id_pair);

 private:
  bool use_point_to_plane;
  double plane_information;
  double plane_dist_threshold;
  double plane_points_dist;
  double infinite_room_min_plane_length;
  double room_min_plane_length, room_max_plane_length;
  double min_plane_points;
  bool use_infinite_room_constraint;
  bool use_room_constraint;

 private:
  std::unique_ptr<PlaneUtils> plane_utils;
  rclcpp::Node::SharedPtr node_obj;
};

}  // namespace s_graphs

#endif  // PLANE_MAPPER_HPP
