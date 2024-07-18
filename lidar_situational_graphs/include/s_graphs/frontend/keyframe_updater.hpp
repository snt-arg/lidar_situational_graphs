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

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace s_graphs {

/**
 * @brief This class decides if a new frame should be registered to the pose graph as a
 * keyframe
 */
class KeyframeUpdater {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor for class KeyframeUpdater
   *
   * @param node
   */
  KeyframeUpdater(const rclcpp::Node::SharedPtr node)
      : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
    keyframe_delta_trans =
        node->get_parameter("keyframe_delta_trans").get_parameter_value().get<double>();
    keyframe_delta_angle =
        node->get_parameter("keyframe_delta_angle").get_parameter_value().get<double>();

    accum_distance = 0.0;
  }

  /**
   * @brief Decide if a new frame should be registered to the graph
   *
   * @param pose: pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose) {
    // first frame is always registered to the graph
    if (is_first) {
      is_first = false;
      prev_keypose = pose;
      return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle();

    // too close to the previous frame
    if (dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   *
   * @return accumulated distance
   */
  double get_accum_distance() const { return accum_distance; }

  void augment_collected_cloud(Eigen::Matrix4f odom_corrected,
                               pcl::PointCloud<PointT>::Ptr cloud) {
    collected_pose_cloud.push_back(std::make_pair(odom_corrected, cloud));
  }

  std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
  get_collected_pose_cloud() {
    return collected_pose_cloud;
  }

  void reset_collected_pose_cloud() { collected_pose_cloud.clear(); }

 private:
  // parameters
  double keyframe_delta_trans;
  double keyframe_delta_angle;

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
  std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
      collected_pose_cloud;
};

}  // namespace s_graphs

#endif  // KEYFRAME_UPDATOR_HPP
