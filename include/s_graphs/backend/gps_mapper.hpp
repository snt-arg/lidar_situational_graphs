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

#ifndef GPS_MAPPER_HPP
#define GPS_MAPPER_HPP

#include <g2o/types/slam3d/edge_se3.h>

#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/keyframe.hpp>

#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace s_graphs {

/**
 * @brief
 */
class GPSMapper {
 public:
  /**
   * @brief Constructor for class GPSMapper
   *
   * @param private_nh
   */
  GPSMapper(const rclcpp::Node::SharedPtr node);
  ~GPSMapper();

 public:
  bool map_gps_data(
      std::shared_ptr<GraphSLAM>& covisibility_graph,
      std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr>& gps_queue,
      const std::map<int, KeyFrame::Ptr>& keyframes);

 private:
  boost::optional<Eigen::Vector3d> zero_utm;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
};
}  // namespace s_graphs

#endif  // GPS_MAPPER_HPP
