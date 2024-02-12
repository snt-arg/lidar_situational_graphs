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

#ifndef KEYFRAME_MAPPER_HPP
#define KEYFRAME_MAPPER_HPP

#include <g2o/types/slam3d/edge_se3.h>

#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/map_cloud_generator.hpp>
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/common/ros_time_hash.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/loop_detector.hpp>

#include "rclcpp/rclcpp.hpp"

namespace s_graphs {

/**
 * @brief
 */
class KeyframeMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor for class KeyframeMapper
   *
   * @param private_nh
   */
  KeyframeMapper(const rclcpp::Node::SharedPtr node);
  ~KeyframeMapper();

 public:
  /**
   * @brief
   *
   * @param covisibility_graph
   * @param odom2map
   * @param keyframe_queue
   * @param keyframes
   * @param new_keyframes
   * @param anchor_node
   * @param anchor_edge
   * @param keyframe_hash
   * @return
   */
  int map_keyframes(
      std::shared_ptr<GraphSLAM>& covisibility_graph,
      Eigen::Isometry3d odom2map,
      std::deque<KeyFrame::Ptr>& keyframe_queue,
      std::map<int, KeyFrame::Ptr>& keyframes,
      std::deque<KeyFrame::Ptr>& new_keyframes,
      g2o::VertexSE3*& anchor_node,
      g2o::EdgeSE3*& anchor_edge,
      std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash>& keyframe_hash);

  /**
   * @brief
   *
   * @param local_graph
   * @param covisibility_graph
   * @param odom2map
   * @param keyframe_queue
   * @param keyframes
   * @param anchor_node
   * @param anchor_edge
   * @return * void
   */
  void map_keyframes(std::shared_ptr<GraphSLAM>& local_graph,
                     std::shared_ptr<GraphSLAM>& covisibility_graph,
                     const Eigen::Isometry3d& odom2map,
                     std::deque<KeyFrame::Ptr>& keyframe_queue,
                     std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
                     g2o::VertexSE3*& anchor_node,
                     g2o::EdgeSE3*& anchor_edge);

  /**
   * @brief
   *
   * @param local_graph
   * @param keyframe
   * @param anchor_node
   * @param anchor_edge
   * @param use_vertex_size_id
   * @return * void
   */
  void add_anchor_node(std::shared_ptr<GraphSLAM>& local_graph,
                       KeyFrame::Ptr keyframe,
                       g2o::VertexSE3*& anchor_node,
                       g2o::EdgeSE3*& anchor_edge,
                       bool use_vertex_id = false);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param keyframe
   * @param prev_keyframe
   * @return * void
   */
  void remap_delayed_keyframe(std::shared_ptr<GraphSLAM>& covisibility_graph,
                              KeyFrame::Ptr keyframe,
                              KeyFrame::Ptr prev_keyframe);

 private:
  rclcpp::Node::SharedPtr node_obj;
  int max_keyframes_per_update;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<LoopDetector> loop_detector;
};
}  // namespace s_graphs

#endif  // KEYFRAME_MAPPER_HPP
