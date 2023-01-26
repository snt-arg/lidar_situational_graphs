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

#include <pcl/octree/octree_search.h>

#include <s_graphs/map_cloud_generator.hpp>

namespace s_graphs {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(
    const std::vector<KeyFrameSnapshot::Ptr>& keyframes,
    double resolution) const {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for (const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    for (const auto& src_pt : keyframe->cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <= 0.0) return cloud;  // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(
    const Eigen::Matrix4f& pose,
    const pcl::PointCloud<PointT>::Ptr& cloud) const {
  pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());
  map_cloud->reserve(cloud->size());
  for (const auto& src_pt : cloud->points) {
    PointT dst_pt;
    dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
    dst_pt.intensity = src_pt.intensity;
    map_cloud->push_back(dst_pt);

    map_cloud->width = cloud->size();
    map_cloud->height = 1;
    map_cloud->is_dense = false;
  }
  return map_cloud;
}

}  // namespace s_graphs
