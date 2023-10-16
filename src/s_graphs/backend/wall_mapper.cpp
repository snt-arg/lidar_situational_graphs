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

#include <s_graphs/backend/wall_mapper.hpp>

namespace s_graphs {

WallMapper::WallMapper(const rclcpp::Node::SharedPtr node) { node_obj = node; }

WallMapper::~WallMapper() {}

void WallMapper::factor_wall(std::shared_ptr<GraphSLAM>& covisibility_graph,
                             std::vector<s_graphs::msg::PlaneData> x_planes_msg,
                             Eigen::Vector3d x_wall_pose,
                             std::vector<s_graphs::msg::PlaneData> y_planes_msg,
                             Eigen::Vector3d y_wall_pose,
                             std::vector<VerticalPlanes>& x_vert_planes,
                             std::vector<VerticalPlanes>& y_vert_planes) {
  if (x_planes_msg.size() == 2) {
    auto matched_x_plane1 = x_vert_planes.begin();
    auto matched_x_plane2 = x_vert_planes.begin();

    matched_x_plane1 =
        std::find_if(x_vert_planes.begin(),
                     x_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == x_planes_msg[0].id);
    matched_x_plane2 =
        std::find_if(x_vert_planes.begin(),
                     x_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == x_planes_msg[1].id);

    if (!(*matched_x_plane1).on_wall && !(*matched_x_plane2).on_wall) {
      add_wall_node_and_edge(
          covisibility_graph, x_wall_pose, *matched_x_plane1, *matched_x_plane2);
    }
  } else if (y_planes_msg.size() == 2) {
    auto matched_y_plane1 = y_vert_planes.begin();
    auto matched_y_plane2 = y_vert_planes.begin();

    matched_y_plane1 =
        std::find_if(y_vert_planes.begin(),
                     y_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == y_planes_msg[0].id);
    matched_y_plane2 =
        std::find_if(y_vert_planes.begin(),
                     y_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == y_planes_msg[1].id);

    if (!(*matched_y_plane1).on_wall && !(*matched_y_plane2).on_wall) {
      add_wall_node_and_edge(
          covisibility_graph, y_wall_pose, *matched_y_plane1, *matched_y_plane2);
    }
  } else {
    std::cout << "Message size is not 2 !! " << std::endl;
  }
}

void WallMapper::add_wall_node_and_edge(std::shared_ptr<GraphSLAM>& covisibility_graph,
                                        Eigen::Vector3d wall_pose,
                                        VerticalPlanes& plane1,
                                        VerticalPlanes& plane2) {
  g2o::VertexWallXYZ* wall_node = covisibility_graph->add_wall_node(wall_pose);
  Eigen::Matrix<double, 3, 3> information_wall_surfaces;
  information_wall_surfaces.setZero();
  information_wall_surfaces(0, 0) = 1e10;
  information_wall_surfaces(1, 1) = 1e10;
  information_wall_surfaces(2, 2) = 1e10;
  auto wall_edge = covisibility_graph->add_wall_2planes_edge(wall_node,
                                                             (plane1).plane_node,
                                                             (plane2).plane_node,
                                                             wall_pose,
                                                             information_wall_surfaces);
  covisibility_graph->add_robust_kernel(wall_edge, "Huber", 1.0);
  (plane1).on_wall = true;
  (plane2).on_wall = true;
}
}  // namespace s_graphs