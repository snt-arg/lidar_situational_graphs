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
#include <std_msgs/ColorRGBA.h>

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
#include <pcl/common/impl/io.hpp>

#include <s_graphs/plane_utils.hpp>

namespace s_graphs {

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGBNormal PointNormal;

/**
 * @brief this class provides tools for different analysis over pointclouds to extract planar surfaces
 */
class PlaneAnalyzer {
public:
  /**
   * @brief
   *
   * @param
   * @return
   */
  PlaneAnalyzer(ros::NodeHandle private_nh);
  ~PlaneAnalyzer();

public:
  /**
   * @brief
   *
   * @param
   * @return
   */
  std::vector<sensor_msgs::PointCloud2> get_segmented_planes(const pcl::PointCloud<PointT>::ConstPtr cloud);

private:
  /**
   * @brief
   *
   * @param
   * @return
   */
  void init_ros(ros::NodeHandle private_nh);

private:
  ros::Publisher segmented_cloud_pub_;

private:
  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr compute_clusters(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr shadow_filter(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  /**
   * @brief Maps an input h from a value between 0.0 and 1.0 into a rainbow.
   * References OctomapProvider in octomap
   *
   * @param
   * @return
   */
  std_msgs::ColorRGBA rainbow_color_map(double h);

  /**
   * @brief
   *
   * @param
   * @return
   */
  std_msgs::ColorRGBA random_color();

private:
  int min_seg_points_;
  int min_horizontal_inliers_, min_vertical_inliers_;
  bool use_euclidean_filter_, use_shadow_filter_;
  std::string plane_extraction_frame_, plane_visualization_frame_;
};
}  // namespace s_graphs

#endif  // PLANE_ANALYZER_HPP
