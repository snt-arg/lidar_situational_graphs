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

#ifndef PLANE_UTILS_HPP
#define PLANE_UTILS_HPP

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

#include <Eigen/Dense>
#include <g2o/edge_se3_plane.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "s_graphs/msg/planes_data.hpp"

namespace s_graphs {

/**
 * @brief
 */
struct plane_data_list {
  plane_data_list() {}
  g2o::Plane3D plane_unflipped;
  int plane_id;
};

/**
 * @brief
 *
 * @param
 * @return
 */
struct structure_data_list {
  plane_data_list plane1;
  plane_data_list plane2;
};

/**
 * @brief
 */
class PlaneUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  PlaneUtils();

 public:
  enum plane_class : uint8_t {
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
    HORT_PLANE = 2,
  };

  /**
   * @brief
   *
   * @param
   * @return
   */
  float width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  float width_between_planes(s_graphs::msg::PlaneData& plane1,
                             s_graphs::msg::PlaneData& plane2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  void correct_plane_direction(int plane_type, s_graphs::msg::PlaneData& plane);

  /**
   * @brief
   *
   * @param
   * @return
   */
  void correct_plane_direction(int plane_type, Eigen::Vector4d& plane);

  /* @brief
   *
   * @param
   * @return
   */
  Eigen::Quaterniond euler_to_quaternion(const double roll,
                                         const double pitch,
                                         const double yaw);

  /**
   * @brief
   *
   * @param
   * @return
   */
  Eigen::Vector3d room_center(const Eigen::Vector4d& x_plane1,
                              const Eigen::Vector4d& x_plane2,
                              const Eigen::Vector4d& y_plane1,
                              const Eigen::Vector4d& y_plane2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  geometry_msgs::msg::Pose room_center(const s_graphs::msg::PlaneData& x_plane1,
                                       const s_graphs::msg::PlaneData& x_plane2,
                                       const s_graphs::msg::PlaneData& y_plane1,
                                       const s_graphs::msg::PlaneData& y_plane2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg,
                     pcl::PointXY& p1,
                     pcl::PointXY& p2,
                     g2o::VertexSE3* keyframe_node);

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg,
                     pcl::PointXY& p1,
                     pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointXY convert_point_to_map(pcl::PointXY point_local,
                                    Eigen::Matrix4d keyframe_pose);

  /**
   * @brief
   *
   * @param
   * @return
   */
  float get_min_segment(const pcl::PointCloud<PointNormal>::Ptr& cloud_1,
                        const pcl::PointCloud<PointNormal>::Ptr& cloud_2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_point_neighbours(const pcl::PointCloud<PointNormal>::Ptr& cloud_1,
                              const pcl::PointCloud<PointNormal>::Ptr& cloud_2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool compute_point_difference(const double plane1_point, const double plane2_point);

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_dot_product(const s_graphs::msg::PlaneData& plane1,
                          const s_graphs::msg::PlaneData& plane2);

  bool plane_dot_product(g2o::VertexPlane* plane1, g2o::VertexPlane* plane2);
  geometry_msgs::msg::Pose extract_infite_room_center(int plane_type,
                                                      pcl::PointXY p1,
                                                      pcl::PointXY p2,
                                                      s_graphs::msg::PlaneData plane1,
                                                      s_graphs::msg::PlaneData plane2,
                                                      Eigen::Vector2d& cluster_center);

  /**
   * @brief
   *
   * @param
   * @return
   */
  double plane_difference(g2o::Plane3D plane1, g2o::Plane3D plane2);
};
}  // namespace s_graphs
#endif  // PLANE_UTILS_HPP
