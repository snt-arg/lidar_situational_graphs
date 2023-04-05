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

#ifndef ROOMS_HPP
#define ROOMS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <visualization_msgs/msg/marker_array.hpp>

namespace g2o {
class VertexRoom;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id
 * @param plane_x1, plane_x2, plane_y1, plane_y2
 * @param plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @param neighbour_ids
 * @param node
 * @param sub_room
 * @param cluster_array
 */
class Rooms {
 public:
  Rooms() {
    plane_x1_node = plane_x2_node = plane_y1_node = plane_y2_node = nullptr;
    node = nullptr;
    // local_graph = std::make_shared<GraphSLAM>();
  }

 public:
  int id;
  g2o::Plane3D plane_x1, plane_x2, plane_y1, plane_y2;
  int plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id;
  bool sub_room;
  visualization_msgs::msg::MarkerArray cluster_array;

  g2o::VertexPlane *plane_x1_node, *plane_x2_node, *plane_y1_node, *plane_y2_node;
  g2o::VertexRoom* node;  // node instance in covisibility graph
  // std::shared_ptr<GraphSLAM> local_graph;
};

}  // namespace s_graphs
#endif  // ROOMS_HPP
