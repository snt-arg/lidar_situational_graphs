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

int WallMapper::factor_wall(std::shared_ptr<GraphSLAM>& graph_slam,
                            std::vector<s_graphs::msg::PlaneData> x_planes_msg,
                            std::vector<s_graphs::msg::PlaneData> y_planes_msg,
                            std::vector<VerticalPlanes>& x_vert_planes,
                            std::vector<VerticalPlanes>& y_vert_planes) {
  if (x_planes_msg.size() == 2) {
  } else if (y_planes_msg.size() == 2) {
  } else {
    std::cout << "Message size is not 2 !! " << std::endl;
  }
  return 0;
}

std::pair<int, int> WallMapper::associate_wall(
    const int& plane_type,
    const KeyFrame::Ptr& keyframe,
    const g2o::Plane3D& det_plane,
    const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    const std::vector<HorizontalPlanes>& hort_planes) {
  std::pair<int, int> data_association;
  return data_association;
}
}  // namespace s_graphs