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

#ifndef MAP_CLOUD_GENERATOR_HPP
#define MAP_CLOUD_GENERATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <s_graphs/common/keyframe.hpp>
#include <vector>

namespace s_graphs {

/**
 * @brief Class that generates a map point cloud from registered keyframes
 */
class MapCloudGenerator {
 public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief Contructor of class MapCloudGenerator
   */
  MapCloudGenerator();
  ~MapCloudGenerator();

  /**
   * @brief Generates a map point cloud
   *
   * @param keyframes
   *          snapshots of keyframes
   * @param resolution
   *          resolution of generated map
   * @return generated map point cloud
   */
  pcl::PointCloud<PointT>::Ptr generate(
      const std::vector<KeyFrameSnapshot::Ptr>& keyframes,
      double resolution) const;
  pcl::PointCloud<PointT>::Ptr generate(
      const Eigen::Matrix4f& pose,
      const pcl::PointCloud<PointT>::Ptr& cloud) const;
};

}  // namespace s_graphs

#endif  // MAP_POINTCLOUD_GENERATOR_HPP
