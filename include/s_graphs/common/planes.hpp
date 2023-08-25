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

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

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
 * @param keyframe_node
 * @param keyframe_node_vec
 * @param plane_node
 * @param color
 */

using PointNormal = pcl::PointXYZRGBNormal;
class Planes {
 public:
  Planes() {}

  Planes(const Planes& old_plane, const bool deep_copy) {
    *this = old_plane;
    if (deep_copy) {
      keyframe_node = new g2o::VertexSE3();
      keyframe_node->setEstimate(old_plane.keyframe_node->estimate());
      plane_node = new g2o::VertexPlane();
      plane_node->setEstimate(old_plane.plane_node->estimate());
    }
  }
  virtual ~Planes() {}

  Planes& operator=(const Planes& old_plane) {
    id = old_plane.id;
    plane = old_plane.plane;
    cloud_seg_body = old_plane.cloud_seg_body;
    cloud_seg_body_vec = old_plane.cloud_seg_body_vec;
    cloud_seg_map = old_plane.cloud_seg_map;
    covariance = old_plane.covariance;
    keyframe_node_vec = old_plane.keyframe_node_vec;
    rviz_color = old_plane.rviz_color; 
    color = old_plane.color;
    revit_id = old_plane.revit_id;
    keyframe_node = old_plane.keyframe_node;
    plane_node = old_plane.plane_node;

    return *this;
  }

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
  std::vector<g2o::VertexSE3*> keyframe_node_vec;  // vector keyframe node instance
  std::vector<double> color;
  std::size_t rviz_color;

  int revit_id;
  g2o::VertexSE3* keyframe_node = nullptr;  // keyframe node instance
  g2o::VertexPlane* plane_node = nullptr;   // node instance
};

class VerticalPlanes : public Planes {
 public:
  VerticalPlanes() : Planes() {}
  ~VerticalPlanes() {}

  // copy constructor
  VerticalPlanes(const VerticalPlanes& old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  VerticalPlanes& operator=(const VerticalPlanes& old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
};

class HorizontalPlanes : public Planes {
 public:
  HorizontalPlanes() : Planes() {}
  ~HorizontalPlanes() {}

  // copy constructor
  HorizontalPlanes(const HorizontalPlanes& old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  HorizontalPlanes& operator=(const HorizontalPlanes& old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
};

}  // namespace s_graphs

#endif  // PLANES_HPP
