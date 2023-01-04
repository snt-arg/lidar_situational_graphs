// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_ANALYZER_HPP
#define FLOOR_ANALYZER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_ros/transform_listener.h"
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
#include <pcl/common/impl/io.hpp>

#include <s_graphs/plane_utils.hpp>
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"

namespace s_graphs {

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGBNormal PointNormal;
/**
 * @brief Class that provides tools for different analysis over
 * pointclouds to extract the floor surfaces.
 */
class FloorAnalyzer {
public:
  /**
   * @brief Constructor for FloorAnalyzer.
   *
   * @param plane_utils_ptr: a pointer containing utils for planes manipulation.
   */
  FloorAnalyzer(std::shared_ptr<PlaneUtils> plane_utils_ptr);
  ~FloorAnalyzer();

public:
  /**
   * @brief Splits current data into multiples floors nodes if available.
   *
   * @param current_x_vert_planes
   * @param current_y_vert_planes
   * @param floor_plane_candidates_vec
   */
  void perform_floor_segmentation(const std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes, std::vector<s_graphs::msg::PlaneData>& floor_plane_candidates_vec);

private:
  std::shared_ptr<PlaneUtils> plane_utils;
};
}  // namespace s_graphs

#endif  // FLOOR_ANALYZER_HPP
