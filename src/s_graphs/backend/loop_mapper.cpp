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

#include <s_graphs/backend/loop_mapper.hpp>

namespace s_graphs {

LoopMapper::LoopMapper(const rclcpp::Node::SharedPtr node) {
  inf_calclator.reset(new InformationMatrixCalculator(node));
}

LoopMapper::~LoopMapper() {}

void LoopMapper::add_loops(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                           const std::vector<Loop::Ptr>& loops,
                           std::mutex& graph_mutex) {
  for (const auto& loop : loops) {
    Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
    Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(
        loop->key1->cloud, loop->key2->cloud, relpose);
    graph_mutex.lock();
    std::cout << "loop found between keyframes " << loop->key1->node->id() << " and "
              << loop->key2->node->id() << std::endl;

    set_data(loop->key1->node);
    set_data(loop->key2->node);

    g2o::EdgeLoopClosure* edge = covisibility_graph->add_loop_closure_edge(
        loop->key1->node, loop->key2->node, relpose, information_matrix);
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    graph_mutex.unlock();
  }
}

void LoopMapper::set_data(g2o::VertexSE3* keyframe_node) {
  auto current_key_data = dynamic_cast<OptimizationData*>(keyframe_node->userData());
  if (current_key_data) {
    current_key_data->set_loop_closure_info(true);
  } else {
    OptimizationData* data = new OptimizationData();
    data->set_loop_closure_info(true);
    keyframe_node->setUserData(data);
  }
}

}  // namespace s_graphs