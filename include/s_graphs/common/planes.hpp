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

#ifndef PLANES_HPP
#define PLANES_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {
/**
 * @brief
 *
 * @param id
 * @param plane
 * @param cloud_seg_body
 * @param cloud_seg_body_vec
 * @param cloud_seg_map
 * @param covariance
 * @param parallel_pair
 * @param keyframe_node
 * @param keyframe_node_vec
 * @param plane_node
 * @param color
 */
struct VerticalPlanes {
 public:
  using PointNormal = pcl::PointXYZRGBNormal;

 public:
  int id;
  g2o::Plane3D plane;
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_body;  // segmented points of the plane in local body frame
  std::vector<pcl::PointCloud<PointNormal>::Ptr>
      cloud_seg_body_vec;  // vector of segmented points of the plane in local body
                           // frame
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_map;           // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;  // covariance of the landmark
  bool parallel_pair;          // checking if the plane has parallel pair
  std::vector<g2o::VertexSE3*> keyframe_node_vec;  // vector keyframe node instance
  std::vector<double> color;
  int revit_id;
  g2o::VertexSE3* keyframe_node = nullptr;  // keyframe node instance
  g2o::VertexPlane* plane_node = nullptr;   // node instance
};

/**
 * @brief
 *
 * @param id
 * @param plane
 * @param cloud_seg_body
 * @param cloud_seg_body_vec
 * @param cloud_seg_map
 * @param covariance
 * @param parallel_pair
 * @param keyframe_node
 * @param keyframe_node_vec
 * @param plane_node
 * @param color
 */
struct HorizontalPlanes {
 public:
  using PointNormal = pcl::PointXYZRGBNormal;

 public:
  int id;
  g2o::Plane3D plane;
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_body;  // segmented points of the plane in local body frame
  std::vector<pcl::PointCloud<PointNormal>::Ptr>
      cloud_seg_body_vec;  // vector of segmented points of the plane in local body
                           // frame
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_map;           // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;  // covariance of the landmark
  bool parallel_pair;          // checking if the plane has parallel pair
  std::vector<g2o::VertexSE3*> keyframe_node_vec;  // vector keyframe node instance
  std::vector<double> color;
  g2o::VertexSE3* keyframe_node = nullptr;  // keyframe node instance
  g2o::VertexPlane* plane_node = nullptr;   // node instance
};

}  // namespace s_graphs

#endif  // PLANES_HPP
