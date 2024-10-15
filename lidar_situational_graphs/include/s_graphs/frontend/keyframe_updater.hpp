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
      : is_first(true),
        prev_keypose(Eigen::Isometry3d::Identity()),
        standing_still(false) {
    keyframe_delta_trans =
        node->get_parameter("keyframe_delta_trans").get_parameter_value().get<double>();
    keyframe_delta_angle =
        node->get_parameter("keyframe_delta_angle").get_parameter_value().get<double>();

    stand_still_time =
        node->get_parameter("stand_still_time").get_parameter_value().get<double>();
    stand_still_delta =
        node->get_parameter("stand_still_delta").get_parameter_value().get<double>();

    accum_distance = 0.0;
    n_measurement = 0;
  }

  /**
   * @brief Decide if a new frame should be registered to the graph
   *
   * @param pose: pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose) {
    auto now = std::chrono::steady_clock::now();

    if (is_first) {
      is_first = false;
      prev_keypose = pose;
      prev_pose = pose;
      prev_time = now;
      n_measurement = 0;
      return true;
    }

    // Calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle();

    // Too close to the previous frame
    if (dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      n_measurement += 1;

      if (n_measurement >= 20) {
        Eigen::Isometry3d prev_delta = prev_pose.inverse() * pose;
        double prev_dx = prev_delta.translation().norm();
        double prev_da = Eigen::AngleAxisd(prev_delta.linear()).angle();

        if (prev_dx < stand_still_delta && prev_da < stand_still_delta) {
          if (!standing_still) {
            standing_still = true;
            still_time = now;
          } else {
            double standing_time =
                std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                          still_time)
                    .count();
            if (standing_time > stand_still_time) {
              Eigen::Isometry3d delta_still = prev_keypose.inverse() * pose;
              double dx_still = delta_still.translation().norm();
              double da_still = Eigen::AngleAxisd(delta_still.linear()).angle();

              if (dx_still >= 0.3 || da_still >= 0.3) {
                prev_keypose = pose;
                prev_pose = pose;
                standing_still = false;
                prev_time = now;

                return true;
              }
            }
          }
        } else
          standing_still = false;

        prev_pose = pose;
        n_measurement = 0;
      }

      prev_time = now;
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    prev_pose = pose;
    standing_still = false;
    prev_time = now;
    n_measurement = 0;

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
  double stand_still_time;
  double stand_still_delta;

  bool is_first, standing_still;
  double accum_distance;
  int n_measurement;
  Eigen::Isometry3d prev_keypose, prev_pose;
  std::vector<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
      collected_pose_cloud;
  std::chrono::steady_clock::time_point prev_time, still_time;
};

}  // namespace s_graphs

#endif  // KEYFRAME_UPDATOR_HPP
