// SPDX-License-Identifier: BSD-2-Clause

#ifndef PLANE_ANALYZER_HPP
#define PLANE_ANALYZER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/features/normal_3d.h>

#include <s_graphs/plane_utils.hpp>

namespace s_graphs {

typedef pcl::PointXYZRGBNormal PointT;
/**
 * @brief this class provides tools for different analysis over pointclouds to extract planar surfaces
 */
class PlaneAnalyzer {
public:
  PlaneAnalyzer(ros::NodeHandle private_nh);
  ~PlaneAnalyzer();

public:
  std::vector<sensor_msgs::PointCloud2> get_segmented_planes(pcl::PointCloud<PointT>::Ptr transformed_cloud);

private:
  void init_ros(ros::NodeHandle private_nh);

private:
  ros::Publisher segmented_cloud_pub_;

private:
  pcl::PointCloud<PointT>::Ptr compute_clusters(const pcl::PointCloud<PointT>::Ptr& extracted_cloud);
  pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(const pcl::PointCloud<PointT>::Ptr& extracted_cloud);
  pcl::PointCloud<PointT>::Ptr shadow_filter(const pcl::PointCloud<PointT>::Ptr& extracted_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

private:
  int min_seg_points_;
  int min_horizontal_inliers_, min_vertical_inliers_;
  bool use_euclidean_filter_, use_shadow_filter_;
  std::string plane_extraction_frame_, plane_visualization_frame_;
};
}  // namespace s_graphs

#endif  // PLANE_ANALYZER_HPP
