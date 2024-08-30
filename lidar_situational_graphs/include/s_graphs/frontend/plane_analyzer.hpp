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

#ifndef PLANE_ANALYZER_HPP
#define PLANE_ANALYZER_HPP

#include <math.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <pcl/common/impl/io.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <string>

#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/common/plane_utils.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace s_graphs {

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGBNormal PointNormal;

/**
 * @brief This class provides tools for different analysis over pointclouds to extract
 * planar surfaces.
 */
class PlaneAnalyzer {
 public:
  /**
   * @brief Constructor of class PlaneAnalyzer
   *
   * @param node
   */
  PlaneAnalyzer(rclcpp::Node::SharedPtr node);
  ~PlaneAnalyzer();

 public:
  /**
   * @brief
   *
   * @param cloud
   * @return Segmented planes found in the point cloud
   */
  std::vector<pcl::PointCloud<PointNormal>::Ptr> extract_segmented_planes(
      const pcl::PointCloud<PointT>::ConstPtr cloud);

 private:
  /**
   * @brief
   *
   * @param extracted_cloud
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr clean_clusters(
      const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @return * std::vector<pcl::PointCloud<PointNormal>::Ptr>
   */
  std::vector<pcl::PointCloud<PointNormal>::Ptr> seperate_clusters(
      const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @return pcl::PointCloud<PointNormal>::Ptr
   */
  std::vector<pcl::PointIndices> computer_clusters(
      const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud,
      const double cluster_tolerance);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @return
   */
  pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(
      const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud);

  /**
   * @brief
   *
   * @param extracted_cloud
   * @param cloud_normals
   * @return
   */
  pcl::PointCloud<PointNormal>::Ptr shadow_filter(
      const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud,
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

  /**
   * @brief
   *
   * @param extracted_cloud_vec
   * @return std::vector<pcl::PointCloud<PointNormal>::Ptr>
   */
  std::vector<pcl::PointCloud<PointNormal>::Ptr> merge_close_planes(
      std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec);

  /**
   * @brief Maps an input h from a value between 0.0 and 1.0 into a rainbow.
   * References OctomapProvider in octomap.
   *
   * @param h
   *          Value between 0.0 and 1.0.
   * @return RGB color
   */
  std_msgs::msg::ColorRGBA rainbow_color_map(double h);

  /**
   * @brief Generates a random RGB color.
   *
   * @return RGB color
   */
  std_msgs::msg::ColorRGBA random_color();

 private:
  int min_seg_points, cluster_min_size;
  int plane_ransac_itr;
  double plane_ransac_acc, cluster_clean_tolerance, cluster_seperation_tolerance,
      plane_merging_tolerance;
  int min_horizontal_inliers, min_vertical_inliers;
  bool use_euclidean_filter, use_shadow_filter;
  std::string plane_extraction_frame, plane_visualization_frame;

  bool save_timings;
  std::ofstream time_recorder;
};
}  // namespace s_graphs

#endif  // PLANE_ANALYZER_HPP
