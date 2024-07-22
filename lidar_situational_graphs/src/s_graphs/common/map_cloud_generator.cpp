#include <pcl/octree/octree_search.h>

#include <s_graphs/common/map_cloud_generator.hpp>

namespace s_graphs {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(
    const std::vector<KeyFrame::Ptr>& keyframes,
    double resolution,
    Eigen::Matrix4f map_floor_t,
    bool use_dense_cloud) const {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  // cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for (const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose =
        map_floor_t * keyframe->node->estimate().matrix().cast<float>();

    pcl::PointCloud<PointT>::Ptr kf_cloud;
    if (use_dense_cloud) {
      kf_cloud = keyframe->dense_cloud;
    } else {
      kf_cloud = keyframe->cloud;
    }

    for (const auto& src_pt : kf_cloud->points) {
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

  downsample_cloud(cloud, resolution);

  return cloud;
}

void MapCloudGenerator::downsample_cloud(pcl::PointCloud<PointT>::Ptr cloud,
                                         double resolution) const {
  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  octree.getOccupiedVoxelCenters(cloud->points);

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;
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

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate_kf_cloud(
    const Eigen::Matrix4f& kf_pose,
    std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
        pose_map_cloud) {
  pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());

  for (int i = 0; i < pose_map_cloud.size(); i += 2) {
    Eigen::Matrix4f current_odom_pose = pose_map_cloud[i].first;
    pcl::PointCloud<PointT>::Ptr cloud = pose_map_cloud[i].second;
    Eigen::Matrix4f odom_pose_transformed = kf_pose.inverse() * current_odom_pose;

    for (const auto& src_pt : cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = odom_pose_transformed * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity;
      map_cloud->push_back(dst_pt);
    }
  }

  return map_cloud;
}

}  // namespace s_graphs
