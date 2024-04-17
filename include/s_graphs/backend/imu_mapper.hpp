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

#ifndef IMU_MAPPER_HPP
#define IMU_MAPPER_HPP

#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/keyframe.hpp>

#include "g2o/edge_se3_priorquat.hpp"
#include "g2o/edge_se3_priorvec.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"

namespace s_graphs {

/**
 * @brief
 */
class IMUMapper {
 public:
  /**
   * @brief Constructor for class IMUMapper
   *
   * @param private_nh
   */
  IMUMapper(const rclcpp::Node::SharedPtr node);
  ~IMUMapper();

 public:
  bool map_imu_data(std::shared_ptr<GraphSLAM>& graph_slam,
                    const std::unique_ptr<tf2_ros::Buffer>& tf_buffer,
                    std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_queue,
                    const std::map<int, KeyFrame::Ptr>& keyframes,
                    const std::string base_frame_id);

 private:
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
};
}  // namespace s_graphs

#endif  // IMU_MAPPER_HPP
