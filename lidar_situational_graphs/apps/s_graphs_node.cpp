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

#include "s_graphs/common/s_graphs.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor multi_executor;
  std::shared_ptr<s_graphs::SGraphsNode> s_graphs_node =
      std::make_shared<s_graphs::SGraphsNode>();

  s_graphs_node->declare_parameter("enable_optimization_timer", true);
  s_graphs_node->declare_parameter("enable_keyframe_timer", true);
  s_graphs_node->declare_parameter("enable_map_publish_timer", true);

  s_graphs_node->start_timers(s_graphs_node->get_parameter("enable_optimization_timer")
                                  .get_parameter_value()
                                  .get<bool>(),
                              s_graphs_node->get_parameter("enable_keyframe_timer")
                                  .get_parameter_value()
                                  .get<bool>(),
                              s_graphs_node->get_parameter("enable_map_publish_timer")
                                  .get_parameter_value()
                                  .get<bool>());

  multi_executor.add_node(s_graphs_node);
  multi_executor.spin();
  rclcpp::shutdown();
  return 0;
}
