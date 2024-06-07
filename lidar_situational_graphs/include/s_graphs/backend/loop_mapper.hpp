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

#ifndef LOOP_MAPPER_HPP
#define LOOP_MAPPER_HPP

#include <g2o/edge_loop_closure.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/frontend/loop_detector.hpp>

namespace s_graphs {

class LoopMapper {
 public:
  LoopMapper(const rclcpp::Node::SharedPtr node);
  ~LoopMapper();

 public:
  void add_loops(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                 const std::vector<Loop::Ptr>& loops,
                 std::mutex& graph_mutex);

 private:
  void set_data(g2o::VertexSE3* keyframe_node);
  bool get_floor_data(g2o::VertexSE3* keyframe_node);

 private:
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};
}  // namespace s_graphs

#endif #LOOP_MAPPER_HPP