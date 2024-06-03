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

#ifndef INFINITE_ROOMS_HPP
#define INFINITE_ROOMS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>

#include "visualization_msgs/msg/marker_array.hpp"

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief Struct that holds information about an infinite room (infinite_room).
 *
 * @var id: Unique Id of the infinite room.
 * @var connected_id
 * @var connected_neighbour_ids
 * @var plane1, plane2: Planes that form the inifite room
 * @var plane1_id, plane2_id: Planes unique ids
 * @var plane1_node, plane2_node
 * @var cluster_center_node
 * @var node
 * @var cluster_array
 */
class InfiniteRooms {
 public:
  InfiniteRooms() {}

  InfiniteRooms(const InfiniteRooms& old_room, const bool deep_copy = false) {
    *this = old_room;

    if (deep_copy) {
      node = new g2o::VertexRoom();
      node->setEstimate(old_room.node->estimate());

      cluster_center_node = new g2o::VertexRoom();
      cluster_center_node->setEstimate(old_room.cluster_center_node->estimate());

      plane1_node = new g2o::VertexPlane();
      plane1_node->setEstimate(old_room.plane1_node->estimate());
      plane2_node = new g2o::VertexPlane();
      plane2_node->setEstimate(old_room.plane2_node->estimate());
    }
  }

  InfiniteRooms& operator=(const InfiniteRooms& old_room) {
    id = old_room.id;
    plane1 = old_room.plane1;
    plane2 = old_room.plane2;
    plane1_id = old_room.plane1_id;
    plane2_id = old_room.plane2_id;
    sub_infinite_room = old_room.sub_infinite_room;
    cluster_array = old_room.cluster_array;

    plane1_node = old_room.plane1_node;
    plane2_node = old_room.plane2_node;
    cluster_center_node = old_room.cluster_center_node;
    node = old_room.node;
    local_graph = old_room.local_graph;

    return *this;
  }

 public:
  int id;
  g2o::Plane3D plane1;
  g2o::Plane3D plane2;
  int plane1_id;
  int plane2_id;
  bool sub_infinite_room;
  visualization_msgs::msg::MarkerArray cluster_array;

  g2o::VertexPlane* plane1_node = nullptr;
  g2o::VertexPlane* plane2_node = nullptr;
  g2o::VertexRoom* cluster_center_node;
  g2o::VertexRoom* node = nullptr;  // node instance in covisibility graph
  std::shared_ptr<GraphSLAM> local_graph;
};

}  // namespace s_graphs
#endif  // INFINITE_ROOMS_HPP
