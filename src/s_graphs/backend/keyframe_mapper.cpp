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

#include <s_graphs/backend/keyframe_mapper.hpp>

namespace s_graphs {

KeyframeMapper::KeyframeMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;

  node_obj->declare_parameter("fix_first_node", false);
  node_obj->declare_parameter("fix_first_node_stddev", "1 1 1 1 1 1");

  max_keyframes_per_update = node_obj->get_parameter("max_keyframes_per_update")
                                 .get_parameter_value()
                                 .get<int>();

  inf_calclator.reset(new InformationMatrixCalculator(node_obj));
  loop_detector.reset(new LoopDetector(node_obj));
}

KeyframeMapper::~KeyframeMapper() {}

int KeyframeMapper::map_keyframes(
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    Eigen::Isometry3d odom2map,
    std::deque<KeyFrame::Ptr>& keyframe_queue,
    std::map<int, KeyFrame::Ptr>& keyframes,
    std::deque<KeyFrame::Ptr>& new_keyframes,
    g2o::VertexSE3*& anchor_node,
    g2o::EdgeSE3*& anchor_edge,
    std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash>& keyframe_hash) {
  int num_processed = 0;
  for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update);
       i++) {
    num_processed = i;

    const auto& keyframe = keyframe_queue[i];
    // new_keyframes will be tested later for loop closure
    new_keyframes.push_back(keyframe);

    // add pose node
    Eigen::Isometry3d odom = odom2map * keyframe->odom;
    keyframe->node = covisibility_graph->add_se3_node(odom);
    keyframe_hash[keyframe->stamp] = keyframe;

    // fix the first node
    if (keyframes.empty() && new_keyframes.size() == 1) {
      add_anchor_node(covisibility_graph, keyframe, anchor_node, anchor_edge);
    }

    if (i == 0 && keyframes.empty()) {
      continue;
    }

    // add edge between consecutive keyframes
    const auto& prev_keyframe =
        i == 0 ? keyframes.rbegin()->second : keyframe_queue[i - 1];

    Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
        keyframe->cloud, prev_keyframe->cloud, relative_pose);
    auto edge = covisibility_graph->add_se3_edge(
        keyframe->node, prev_keyframe->node, relative_pose, information);
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
  }
  return num_processed;
}

void KeyframeMapper::map_keyframes(std::shared_ptr<GraphSLAM>& local_graph,
                                   std::shared_ptr<GraphSLAM>& covisibility_graph,
                                   const Eigen::Isometry3d& odom2map,
                                   std::deque<KeyFrame::Ptr>& keyframe_queue,
                                   std::map<int, s_graphs::KeyFrame::Ptr>& keyframes,
                                   g2o::VertexSE3*& anchor_node,
                                   g2o::EdgeSE3*& anchor_edge) {
  int num_processed = 0;
  for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update);
       i++) {
    num_processed = i;
    const auto& keyframe = keyframe_queue[i];

    // add pose node
    keyframe->node = local_graph->copy_se3_node(keyframe->node);

    if (i == 0 && keyframes.empty()) {
      keyframes.insert({keyframe->id(), keyframe});
      // add_anchor_node(local_graph, keyframe, anchor_node, anchor_edge, true);
      continue;
    }

    keyframes.insert({keyframe->id(), keyframe});

    // add edge between consecutive keyframes
    const auto& prev_keyframe =
        i == 0 ? (keyframes.rbegin()->second) : keyframe_queue[i - 1];

    bool edge_found = false;
    for (g2o::HyperGraph::EdgeSet::iterator it =
             covisibility_graph->graph->edges().begin();
         it != covisibility_graph->graph->edges().end();
         ++it) {
      g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);

      if (edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
            covisibility_graph->graph->vertices().at(edge_se3->vertices()[0]->id()));
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
            covisibility_graph->graph->vertices().at(edge_se3->vertices()[1]->id()));

        if (v1->id() == keyframe->id() && v2->id() == prev_keyframe->id()) {
          auto edge =
              local_graph->copy_se3_edge(edge_se3, keyframe->node, prev_keyframe->node);
          local_graph->add_robust_kernel(edge, "Huber", 1.0);
          edge_found = true;
          break;
        }
      }
    }

    if (!edge_found) {
      Eigen::Matrix4f loop_relative_pose;
      bool loop_found =
          loop_detector->matching(keyframe, prev_keyframe, loop_relative_pose);

      if (0) {
        Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
            keyframe->cloud,
            prev_keyframe->cloud,
            Eigen::Isometry3d(loop_relative_pose.cast<double>()));

        auto edge = local_graph->add_se3_edge(
            keyframe->node,
            prev_keyframe->node,
            Eigen::Isometry3d(loop_relative_pose.cast<double>()),
            information);
        local_graph->add_robust_kernel(edge, "Huber", 1.0);
      } else {
        Eigen::Isometry3d relative_pose =
            keyframe->node->estimate().inverse() * prev_keyframe->node->estimate();
        Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);

        auto edge = local_graph->add_se3_edge(
            keyframe->node, prev_keyframe->node, relative_pose, information);
        local_graph->add_robust_kernel(edge, "Huber", 1.0);
      }
    }
  }
}

void KeyframeMapper::add_anchor_node(std::shared_ptr<GraphSLAM>& graph,
                                     KeyFrame::Ptr keyframe,
                                     g2o::VertexSE3*& anchor_node,
                                     g2o::EdgeSE3*& anchor_edge,
                                     bool use_vertex_id) {
  if (node_obj->get_parameter("fix_first_node").get_parameter_value().get<bool>()) {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    std::stringstream sst(node_obj->get_parameter("fix_first_node_stddev")
                              .get_parameter_value()
                              .get<std::string>());
    for (int j = 0; j < 6; j++) {
      double stddev = 1.0;
      sst >> stddev;
      inf(j, j) = 1.0 / stddev;
    }

    if (!use_vertex_id)
      anchor_node = graph->add_se3_node(Eigen::Isometry3d::Identity());
    else
      anchor_node = graph->add_se3_node(Eigen::Isometry3d::Identity(), use_vertex_id);
    anchor_node->setFixed(true);
    OptimizationData* data = new OptimizationData();
    data->set_anchor_node_info(true);
    anchor_node->setUserData(data);

    anchor_edge = graph->add_se3_edge(
        anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
  }
}

void KeyframeMapper::remap_delayed_keyframe(
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    KeyFrame::Ptr keyframe,
    KeyFrame::Ptr prev_keyframe) {
  Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
  Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
      keyframe->cloud, prev_keyframe->cloud, relative_pose);

  std::set<g2o::HyperGraph::Edge*> edges = keyframe->node->edges();
  for (const auto& edge : edges) {
    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (edge_se3) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);

      if (v1->id() == prev_keyframe->id() || v2->id() == prev_keyframe->id()) {
        covisibility_graph->update_se3edge_information(edge_se3, information);
      }
    }
  }
}

}  // namespace s_graphs
