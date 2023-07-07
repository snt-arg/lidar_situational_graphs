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

#include <s_graphs/backend/imu_mapper.hpp>

namespace s_graphs {

IMUMapper::IMUMapper(const rclcpp::Node::SharedPtr node) {
  imu_time_offset =
      node->get_parameter("imu_time_offset").get_parameter_value().get<double>();
  enable_imu_orientation =
      node->get_parameter("enable_imu_orientation").get_parameter_value().get<bool>();
  enable_imu_acceleration =
      node->get_parameter("enable_imu_acceleration").get_parameter_value().get<bool>();
  imu_orientation_edge_stddev = node->get_parameter("imu_orientation_edge_stddev")
                                    .get_parameter_value()
                                    .get<double>();
  imu_acceleration_edge_stddev = node->get_parameter("imu_acceleration_edge_stddev")
                                     .get_parameter_value()
                                     .get<double>();
}

IMUMapper::~IMUMapper() {}

bool IMUMapper::map_imu_data(std::shared_ptr<GraphSLAM>& graph_slam,
                             const std::unique_ptr<tf2_ros::Buffer>& tf_buffer,
                             std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_queue,
                             const std::map<int, KeyFrame::Ptr>& keyframes,
                             const std::string base_frame_id) {
  bool updated = false;
  auto imu_cursor = imu_queue.begin();

  for (auto& keyframe : keyframes) {
    if (keyframe.second->stamp > imu_queue.back()->header.stamp) {
      break;
    }

    if (keyframe.second->stamp < (*imu_cursor)->header.stamp ||
        keyframe.second->acceleration) {
      continue;
    }

    // find imu data which is closest to the keyframe
    auto closest_imu = imu_cursor;
    for (auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
      auto dt = (rclcpp::Time((*closest_imu)->header.stamp) - keyframe.second->stamp)
                    .seconds();
      auto dt2 =
          (rclcpp::Time((*imu)->header.stamp) - keyframe.second->stamp).seconds();
      if (std::abs(dt) < std::abs(dt2)) {
        break;
      }

      closest_imu = imu;
    }

    imu_cursor = closest_imu;
    if (0.2 <
        std::abs((rclcpp::Time((*closest_imu)->header.stamp) - keyframe.second->stamp)
                     .seconds())) {
      continue;
    }

    const auto& imu_ori = (*closest_imu)->orientation;
    const auto& imu_acc = (*closest_imu)->linear_acceleration;

    geometry_msgs::msg::Vector3Stamped acc_imu;
    geometry_msgs::msg::Vector3Stamped acc_base;
    geometry_msgs::msg::QuaternionStamped quat_imu;
    geometry_msgs::msg::QuaternionStamped quat_base;

    quat_imu.header.frame_id = acc_imu.header.frame_id =
        (*closest_imu)->header.frame_id;
    quat_imu.header.stamp = acc_imu.header.stamp = rclcpp::Time(0);
    acc_imu.vector = (*closest_imu)->linear_acceleration;
    quat_imu.quaternion = (*closest_imu)->orientation;

    try {
      tf_buffer->transform(acc_imu, acc_base, base_frame_id);
      tf_buffer->transform(quat_imu, quat_base, base_frame_id);
    } catch (std::exception& e) {
      std::cerr << "failed to find transform!!" << std::endl;
      return false;
    }

    keyframe.second->acceleration =
        Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
    keyframe.second->orientation = Eigen::Quaterniond(quat_base.quaternion.w,
                                                      quat_base.quaternion.x,
                                                      quat_base.quaternion.y,
                                                      quat_base.quaternion.z);
    keyframe.second->orientation = keyframe.second->orientation;
    if (keyframe.second->orientation->w() < 0.0) {
      keyframe.second->orientation->coeffs() = -keyframe.second->orientation->coeffs();
    }

    if (enable_imu_orientation) {
      Eigen::MatrixXd info =
          Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
      auto edge = graph_slam->add_se3_prior_quat_edge(
          keyframe.second->node, *keyframe.second->orientation, info);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }

    if (enable_imu_acceleration) {
      Eigen::MatrixXd info =
          Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
      g2o::OptimizableGraph::Edge* edge =
          graph_slam->add_se3_prior_vec_edge(keyframe.second->node,
                                             -Eigen::Vector3d::UnitZ(),
                                             *keyframe.second->acceleration,
                                             info);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }
    updated = true;
  }

  auto remove_loc = std::upper_bound(
      imu_queue.begin(),
      imu_queue.end(),
      keyframes.rbegin()->second->stamp,
      [=](const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::SharedPtr imu) {
        return stamp < imu->header.stamp;
      });
  imu_queue.erase(imu_queue.begin(), remove_loc);

  return updated;
}

}  // namespace s_graphs
