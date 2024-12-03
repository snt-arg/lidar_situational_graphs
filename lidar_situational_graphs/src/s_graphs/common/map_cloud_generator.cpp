#include <pcl/octree/octree_search.h>

#include <s_graphs/common/map_cloud_generator.hpp>

namespace s_graphs {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<PointT>::Ptr MapCloudGenerator::generate(
    const std::vector<KeyFrame::Ptr>& keyframes,
    const double resolution,
    const Eigen::Matrix4f map_floor_t,
    const bool use_dense_cloud) const {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  }

  std::set<PointT, PointComparator> unique_points;
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
#ifdef USE_RGB_CLOUD
      dst_pt.rgb = src_pt.rgb;
#else
      dst_pt.intensity = src_pt.intensity;
#endif
      if (unique_points.insert(dst_pt).second) cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <= 0.0) return cloud;

  pcl::PointCloud<PointT>::Ptr filtered_cloud = downsample_cloud(cloud, resolution);

  return filtered_cloud;
}

pcl::PointCloud<PointT>::Ptr MapCloudGenerator::downsample_cloud(
    const pcl::PointCloud<PointT>::Ptr cloud,
    double resolution) const {
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);

  pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();
  octree.getOccupiedVoxelCenters(filtered_cloud->points);

  for (auto& voxel_point : filtered_cloud->points) {
    PointT point;
    point.x = voxel_point.x;
    point.y = voxel_point.y;
    point.z = voxel_point.z;
    std::vector<int> point_indices;
    octree.voxelSearch(voxel_point, point_indices);

    // Find the original points within the voxel and average their intensity
#ifdef USE_RGB_CLOUD
    float color_sum = 0.0f;

    for (int idx : point_indices) {
      color_sum += cloud->points[idx].rgb;
    }
    voxel_point.rgb = color_sum / point_indices.size();
#else
    float intensity_sum = 0.0f;
    for (int idx : point_indices) {
      intensity_sum += cloud->points[idx].intensity;
    }
    voxel_point.intensity = intensity_sum / point_indices.size();
#endif
  }

  filtered_cloud->width = filtered_cloud->size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = false;

  return filtered_cloud;
}

pcl::PointCloud<PointT>::Ptr MapCloudGenerator::generate(
    const Eigen::Matrix4f& pose,
    const pcl::PointCloud<PointT>::Ptr& cloud) const {
  pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());
  map_cloud->reserve(cloud->size());
  for (const auto& src_pt : cloud->points) {
    PointT dst_pt;
    dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
#ifdef USE_RGB_CLOUD
    dst_pt.rgb = src_pt.rgb;
#else
    dst_pt.intensity = src_pt.intensity;
#endif
    map_cloud->push_back(dst_pt);

    map_cloud->width = cloud->size();
    map_cloud->height = 1;
    map_cloud->is_dense = false;
  }
  return map_cloud;
}

pcl::PointCloud<PointT>::Ptr MapCloudGenerator::generate_floor_cloud(
    const std::vector<KeyFrame::Ptr>& keyframes,
    const int& current_floor_level,
    const double resolution,
    const Eigen::Matrix4f map_floor_t,
    const bool use_dense_cloud) {
  if (keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

  std::vector<KeyFrame::Ptr> floor_keyframes;
  for (const auto& keyframe : keyframes) {
    if (keyframe->floor_level == current_floor_level) {
      floor_keyframes.push_back(keyframe);
    }
  }

  pcl::PointCloud<PointT>::Ptr filtered =
      this->generate(floor_keyframes, resolution, map_floor_t, use_dense_cloud);

  return filtered;
}

pcl::PointCloud<PointT>::Ptr MapCloudGenerator::generate_kf_cloud(
    const int& current_floor_level,
    const Eigen::Matrix4f& kf_pose,
    const std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
        pose_map_cloud,
    const std::map<int, Floors> floors_vec,
    const bool use_floor_color) {
  pcl::PointCloud<PointT>::Ptr map_cloud(new pcl::PointCloud<PointT>());
  std::set<PointT, PointComparator> unique_points;
  auto current_floor = floors_vec.find(current_floor_level);

  for (int i = 0; i < pose_map_cloud.size(); i += 2) {
    Eigen::Matrix4f current_odom_pose = pose_map_cloud[i].first;
    pcl::PointCloud<PointT>::Ptr cloud = pose_map_cloud[i].second;
    Eigen::Matrix4f odom_pose_transformed = kf_pose.inverse() * current_odom_pose;

    for (const auto& src_pt : cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = odom_pose_transformed * src_pt.getVector4fMap();
#ifdef USE_RGB_CLOUD
      if (use_floor_color && current_floor != floors_vec.end() &&
          current_floor->second.color.size() == 3) {
        dst_pt.r = current_floor->second.color[0];
        dst_pt.g = current_floor->second.color[1];
        dst_pt.b = current_floor->second.color[2];
      } else
        dst_pt.rgb = src_pt.rgb;
#else
      dst_pt.intensity = src_pt.intensity;
#endif
      if (unique_points.insert(dst_pt).second) map_cloud->push_back(dst_pt);
    }
  }

  return map_cloud;
}

void MapCloudGenerator::color_cloud_using_floor_color(
    const int& current_floor_level,
    const std::map<int, Floors>& floors_vec,
    const pcl::PointCloud<PointT>::Ptr& cloud) {
#ifdef USE_RGB_CLOUD
  auto current_floor = floors_vec.find(current_floor_level);
  if (current_floor != floors_vec.end() && current_floor->second.color.size() == 3) {
    for (auto& src_pt : cloud->points) {
      src_pt.r = current_floor->second.color[0];
      src_pt.g = current_floor->second.color[1];
      src_pt.b = current_floor->second.color[2];
    }
  }
#else
  return;
#endif
  return;
}

}  // namespace s_graphs
