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

#include <s_graphs/backend/gps_mapper.hpp>

namespace s_graphs {

GPSMapper::GPSMapper(const rclcpp::Node::SharedPtr node) {
  gps_time_offset =
      node->get_parameter("gps_time_offset").get_parameter_value().get<int>();
  gps_edge_stddev_xy =
      node->get_parameter("gps_edge_stddev_xy").get_parameter_value().get<double>();
  gps_edge_stddev_z =
      node->get_parameter("gps_edge_stddev_z").get_parameter_value().get<double>();
}

GPSMapper::~GPSMapper() {}

bool GPSMapper::map_gps_data(
    std::shared_ptr<GraphSLAM>& graph_slam,
    std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr>& gps_queue,
    const std::vector<KeyFrame::Ptr>& keyframes) {
  bool updated = false;
  auto gps_cursor = gps_queue.begin();

  for (auto& keyframe : keyframes) {
    if (keyframe->stamp > gps_queue.back()->header.stamp) {
      break;
    }

    if (keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
      continue;
    }

    // find the gps data which is closest to the keyframe
    auto closest_gps = gps_cursor;
    for (auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
      auto dt =
          (rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds();
      auto dt2 = (rclcpp::Time((*gps)->header.stamp) - keyframe->stamp).seconds();
      if (std::abs(dt) < std::abs(dt2)) {
        break;
      }

      closest_gps = gps;
    }

    // if the time residual between the gps and keyframe is too large, skip it
    gps_cursor = closest_gps;
    if (0.2 <
        std::abs(
            (rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds())) {
      continue;
    }

    // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM
    // coordinate
    geodesy::UTMPoint utm;
    geodesy::fromMsg((*closest_gps)->position, utm);
    Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

    // the first gps data position will be the origin of the map
    if (!zero_utm) {
      zero_utm = xyz;
    }
    xyz -= (*zero_utm);

    keyframe->utm_coord = xyz;

    g2o::OptimizableGraph::Edge* edge;
    if (std::isnan(xyz.z())) {
      Eigen::Matrix2d information_matrix =
          Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
      edge = graph_slam->add_se3_prior_xy_edge(
          keyframe->node, xyz.head<2>(), information_matrix);
    } else {
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
      information_matrix(2, 2) /= gps_edge_stddev_z;
      edge =
          graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
    }
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);

    updated = true;

    auto remove_loc = std::upper_bound(
        gps_queue.begin(),
        gps_queue.end(),
        keyframes.back()->stamp,
        [=](const rclcpp::Time& stamp,
            const geographic_msgs::msg::GeoPointStamped::SharedPtr& geopoint) {
          return stamp < geopoint->header.stamp;
        });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }
}

}  // namespace s_graphs
