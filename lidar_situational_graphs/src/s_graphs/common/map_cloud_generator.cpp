#include <pcl/octree/octree_search.h>

#include <s_graphs/common/map_cloud_generator.hpp>

namespace s_graphs {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(
    const std::vector<KeyFrame::Ptr>& keyframes,
    double resolution) const {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  // cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for (const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>();
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

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate_floor_cloud(
    const int& current_floor_level,
    double resolution,
    const std::vector<KeyFrame::Ptr>& keyframes) {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

  std::vector<KeyFrame::Ptr> floor_keyframes;
  for (const auto& keyframe : keyframes) {
    if (keyframe->floor_level == current_floor_level) {
      floor_keyframes.push_back(keyframe);
    }
  }

  pcl::PointCloud<PointT>::Ptr filtered = this->generate(floor_keyframes, resolution);

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
