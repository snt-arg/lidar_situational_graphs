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
 * @brief This class provides tools for different analysis over pointclouds to extract planar surfaces.
 */
class PlaneAnalyzer {
public:
  /**
   * @brief Constructor of class PlaneAnalyzer
   *
   * @param private_nh
   */
  PlaneAnalyzer(ros::NodeHandle private_nh);
  ~PlaneAnalyzer();

public:
  /**
   * @brief
   *
   * @param cloud
   * @return Segmented planes found in the point cloud
   */
  std::vector<pcl::PointCloud<PointNormal>::Ptr> extract_segmented_planes(const pcl::PointCloud<PointT>::ConstPtr cloud);

private:
  /**
   * @brief Initializes ros related jobs.
   *
   * @param private_nh
   */
  void init_ros(ros::NodeHandle private_nh);

private:
  ros::Publisher segmented_cloud_pub;

private:
  /**
   * @brief
   *
   * @param extracted_cloud
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr compute_clusters(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @return
   */
  pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @param cloud_normals
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr shadow_filter(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  /**
   * @brief Maps an input h from a value between 0.0 and 1.0 into a rainbow.
   * References OctomapProvider in octomap.
   *
   * @param h
   *          Value between 0.0 and 1.0.
   * @return RGB color
   */
  std_msgs::ColorRGBA rainbow_color_map(double h);

  /**
   * @brief Generates a random RGB color.
   *
   * @return RGB color
   */
  std_msgs::ColorRGBA random_color();

private:
  int min_seg_points_;
  int min_horizontal_inliers, min_vertical_inliers;
  bool use_euclidean_filter, use_shadow_filter;
  std::string plane_extraction_frame, plane_visualization_frame;
};
}  // namespace s_graphs

#endif  // PLANE_ANALYZER_HPP
