// SPDX-License-Identifier: BSD-2-Clause

#ifndef MAP_CLOUD_GENERATOR_HPP
#define MAP_CLOUD_GENERATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <s_graphs/keyframe.hpp>
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
