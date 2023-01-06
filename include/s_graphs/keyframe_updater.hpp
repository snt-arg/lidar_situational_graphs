// SPDX-License-Identifier: BSD-2-Clause

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

 private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
};

}  // namespace s_graphs

#endif  // KEYFRAME_UPDATOR_HPP
