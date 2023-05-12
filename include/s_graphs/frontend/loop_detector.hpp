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

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <g2o/types/slam3d/vertex_se3.h>

#include <boost/format.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/registrations.hpp>

namespace s_graphs {

/**
 * @brief Struct Loop
 */
struct Loop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  /**
   * @brief Constructor of struct Loop
   *
   * @param key1
   * @param key2
   * @param relpose
   */
  Loop(const KeyFrame::Ptr& key1,
       const KeyFrame::Ptr& key2,
       const Eigen::Matrix4f& relpose)
      : key1(key1), key2(key2), relative_pose(relpose) {}

 public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief This class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
 public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief Constructor of the class LoopDetector
   *
   * @param node
   */
  LoopDetector(const rclcpp::Node::SharedPtr node) {
    distance_thresh =
        node->get_parameter("distance_thresh").get_parameter_value().get<double>();
    accum_distance_thresh = node->get_parameter("accum_distance_thresh")
                                .get_parameter_value()
                                .get<double>();
    distance_from_last_edge_thresh =
        node->get_parameter("min_edge_interval").get_parameter_value().get<double>();

    fitness_score_max_range = node->get_parameter("fitness_score_max_range")
                                  .get_parameter_value()
                                  .get<double>();
    fitness_score_thresh =
        node->get_parameter("fitness_score_thresh").get_parameter_value().get<double>();

    s_graphs::registration_params params;
    params = {
        node->get_parameter("registration_method")
            .get_parameter_value()
            .get<std::string>(),
        node->get_parameter("reg_num_threads").get_parameter_value().get<int>(),
        node->get_parameter("reg_transformation_epsilon")
            .get_parameter_value()
            .get<double>(),
        node->get_parameter("reg_maximum_iterations").get_parameter_value().get<int>(),
        node->get_parameter("reg_max_correspondence_distance")
            .get_parameter_value()
            .get<double>(),
        node->get_parameter("reg_correspondence_randomness")
            .get_parameter_value()
            .get<int>(),
        node->get_parameter("reg_resolution").get_parameter_value().get<double>(),
        node->get_parameter("reg_use_reciprocal_correspondences")
            .get_parameter_value()
            .get<bool>(),
        node->get_parameter("reg_max_optimizer_iterations")
            .get_parameter_value()
            .get<int>(),
        node->get_parameter("reg_nn_search_method")
            .get_parameter_value()
            .get<std::string>()};

    registration = select_registration_method(params);
    last_edge_accum_distance = 0.0;
  }

  /**
   * @brief Detect loops and add them to the pose graph
   *
   * @param keyframes
   *          Keyframes
   * @param new_keyframes
   *          Newly registered keyframes
   * @param graph_slam
   *          Pose graph
   * @return Loop vector
   */
  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes,
                                const std::deque<KeyFrame::Ptr>& new_keyframes,
                                s_graphs::GraphSLAM& graph_slam) {
    std::vector<Loop::Ptr> detected_loops;
    for (const auto& new_keyframe : new_keyframes) {
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe, graph_slam);
      if (loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes,
                                const std::vector<KeyFrame::Ptr>& new_keyframes,
                                s_graphs::GraphSLAM& graph_slam) {
    std::vector<Loop::Ptr> detected_loops;
    for (const auto& new_keyframe : new_keyframes) {
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe, graph_slam);
      if (loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  std::vector<Loop::Ptr> detectWithAllKeyframes(
      const std::vector<KeyFrame::Ptr>& keyframes,
      const std::vector<KeyFrame::Ptr>& new_keyframes,
      s_graphs::GraphSLAM& graph_slam) {
    std::vector<Loop::Ptr> detected_loops;
    for (const auto& new_keyframe : new_keyframes) {
      auto loop = matching(keyframes, new_keyframe, graph_slam, false);
      if (loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  /**
   * @brief
   *
   * @return Distance treshold
   */
  double get_distance_thresh() const { return distance_thresh; }

 private:
  /**
   * @brief Find loop candidates. A detected loop begins at one of #keyframes and ends
   * at #new_keyframe
   *
   * @param keyframes
   *          Candidate keyframes of loop start
   * @param new_keyframe
   *          Loop end keyframe
   * @return Loop candidates
   */
  std::vector<KeyFrame::Ptr> find_candidates(
      const std::vector<KeyFrame::Ptr>& keyframes,
      const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if (new_keyframe->accum_distance - last_edge_accum_distance <
        distance_from_last_edge_thresh) {
      return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);

    for (const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if (new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if (dist > distance_thresh) {
        continue;
      }

      candidates.push_back(k);
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between
   * keyframes consisting the loop. If they are matched well, the loop is added to the
   * pose graph
   *
   * @param candidate_keyframes
   *          candidate keyframes of loop start
   * @param new_keyframe
   *          loop end keyframe
   * @param graph_slam
   *          graph slam
   * @return Loop pointer
   */
  Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes,
                     const KeyFrame::Ptr& new_keyframe,
                     s_graphs::GraphSLAM& graph_slam,
                     bool use_prior = true) {
    if (candidate_keyframes.empty() || new_keyframe->cloud->points.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    // std::cout << std::endl;
    // std::cout << "--- loop detection ---" << std::endl;
    // std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    // std::cout << "matching" << std::flush;
    auto t1 = rclcpp::Clock{}.now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for (const auto& candidate : candidate_keyframes) {
      if (candidate->cloud->points.empty()) continue;
      registration->setInputSource(candidate->cloud);
      Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
      new_keyframe_estimate.linear() =
          Eigen::Quaterniond(new_keyframe_estimate.linear())
              .normalized()
              .toRotationMatrix();
      Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
      candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear())
                                        .normalized()
                                        .toRotationMatrix();

      if (use_prior) {
        Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate)
                                    .matrix()
                                    .cast<float>();
        guess(2, 3) = 0.0;
        registration->align(*aligned, guess);
      } else {
        Eigen::Matrix4f guess;
        guess << 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 1, 0, 0, 0, 0, 1;
        // std::cout << "Using our guess";
        registration->align(*aligned, guess);
        // registration->align(*aligned);
      }
      // std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if (!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;
      best_matched = candidate;
      relative_pose = registration->getFinalTransformation();
    }

    auto t2 = rclcpp::Clock{}.now();
    // std::cout << " done" << std::endl;
    // std::cout << "best_score: " << boost::format("%.3f") % best_score
    //           << "    time: " << boost::format("%.3f") % (t2 - t1).seconds() <<
    //           "[sec]"
    //           << std::endl;

    if (best_score > fitness_score_thresh) {
      std::cout << "loop not found... BEST SCORE:" << best_score << std::endl;
      return nullptr;
    } else {
      std::cout << "loop found:" << best_score << std::endl;
    }

    // std::cout << "loop found!!" << std::endl;
    // std::cout
    //     << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - "
    //     << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose()
    //     << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }

 private:
  double distance_thresh;  // estimated distance between keyframes consisting a loop
                           // must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one
                                          // at least this distance

  double fitness_score_max_range;  // maximum allowable distance between corresponding
                                   // points
  double fitness_score_thresh;     // threshold for scan matching

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace s_graphs

#endif  // LOOP_DETECTOR_HPP
