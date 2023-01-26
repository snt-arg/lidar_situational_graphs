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

// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_ANALYZER_HPP
#define FLOOR_ANALYZER_HPP

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
#include <s_graphs/plane_utils.hpp>
#include <string>

#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"

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
  void perform_floor_segmentation(
      const std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes,
      const std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes,
      std::vector<s_graphs::msg::PlaneData>& floor_plane_candidates_vec);

 private:
  std::shared_ptr<PlaneUtils> plane_utils;
};
}  // namespace s_graphs

#endif  // FLOOR_ANALYZER_HPP
