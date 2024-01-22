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

#include <s_graphs/backend/floor_mapper.hpp>

namespace s_graphs {

FloorMapper::FloorMapper() {
  // initial floor id will always be set to zero
  int floor_id = 0;
  set_floor_level(floor_id);
}

FloorMapper::~FloorMapper() {}

void FloorMapper::lookup_floors(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const s_graphs::msg::RoomData room_data,
    std::unordered_map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  double floor_threshold = 0.5;

  if (floors_vec.empty())
    factor_floor_node(graph_slam,
                      room_data,
                      floors_vec,
                      rooms_vec,
                      x_infinite_rooms,
                      y_infinite_rooms);

  for (const auto& floor : floors_vec) {
    if (floor.second.id == room_data.id) {
      double floor_dist = sqrt(pow(floor.second.node->estimate().translation()(0) -
                                       room_data.room_center.position.x,
                                   2) +
                               pow(floor.second.node->estimate().translation()(1) -
                                       room_data.room_center.position.y,
                                   2));
      if (floor_dist > floor_threshold) {
        update_floor_node(graph_slam,
                          floor.second.node,
                          room_data,
                          rooms_vec,
                          x_infinite_rooms,
                          y_infinite_rooms);
      }
    } else {
      factor_floor_node(graph_slam,
                        room_data,
                        floors_vec,
                        rooms_vec,
                        x_infinite_rooms,
                        y_infinite_rooms);
    }
  }
}

void FloorMapper::factor_floor_node(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const s_graphs::msg::RoomData room_data,
    std::unordered_map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  g2o::VertexFloor* floor_node;
  Eigen::Isometry3d floor_pose;
  Eigen::Quaterniond floor_quat;
  floor_quat.x() = 0;
  floor_quat.y() = 0;
  floor_quat.z() = 0;
  floor_quat.w() = 1;
  floor_pose.linear() = floor_quat.toRotationMatrix();
  floor_pose.translation().x() = room_data.room_center.position.x;
  floor_pose.translation().y() = room_data.room_center.position.y;
  floor_pose.translation().z() = room_data.room_center.position.z;

  Floors det_floor;
  det_floor.graph_id = graph_slam->retrieve_local_nbr_of_vertices();
  floor_node = graph_slam->add_floor_node(floor_pose);
  det_floor.id = room_data.id;
  det_floor.plane_x1_id = room_data.x_planes[0].id;
  det_floor.plane_x2_id = room_data.x_planes[1].id;
  det_floor.plane_y1_id = room_data.y_planes[0].id;
  det_floor.plane_y2_id = room_data.y_planes[1].id;
  det_floor.node = floor_node;
  floors_vec.insert({det_floor.id, det_floor});

  factor_floor_room_nodes(graph_slam,
                          floor_pose,
                          floor_node,
                          rooms_vec,
                          x_infinite_rooms,
                          y_infinite_rooms);
}

void FloorMapper::update_floor_node(
    std::shared_ptr<GraphSLAM>& graph_slam,
    g2o::VertexFloor* floor_node,
    const s_graphs::msg::RoomData room_data,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Isometry3d floor_pose;
  Eigen::Quaterniond floor_quat;
  floor_quat.x() = 0;
  floor_quat.y() = 0;
  floor_quat.z() = 0;
  floor_quat.w() = 1;
  floor_pose.linear() = floor_quat.toRotationMatrix();
  floor_pose.translation().x() = room_data.room_center.position.x;
  floor_pose.translation().y() = room_data.room_center.position.y;
  floor_pose.translation().z() = room_data.room_center.position.z;

  graph_slam->update_floor_node(floor_node, floor_pose);
  factor_floor_room_nodes(graph_slam,
                          floor_pose,
                          floor_node,
                          rooms_vec,
                          x_infinite_rooms,
                          y_infinite_rooms);
}

void FloorMapper::factor_floor_room_nodes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const Eigen::Isometry3d& floor_pose,
    g2o::VertexFloor* floor_node,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Matrix2d information_floor;
  information_floor(0, 0) = 0.0001;
  information_floor(1, 1) = 0.0001;

  remove_floor_room_nodes(graph_slam, floor_node);

  for (const auto& room : rooms_vec) {
    Eigen::Vector2d measurement;
    measurement(0) =
        floor_pose.translation()(0) - room.second.node->estimate().translation()(0);
    measurement(1) =
        floor_pose.translation()(1) - room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor_node, room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  for (const auto& x_infinite_room : x_infinite_rooms) {
    Eigen::Vector2d measurement;
    measurement(0) = floor_pose.translation()(0) -
                     x_infinite_room.second.node->estimate().translation()(0);
    measurement(1) = floor_pose.translation()(1) -
                     x_infinite_room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor_node, x_infinite_room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  for (const auto& y_infinite_room : y_infinite_rooms) {
    Eigen::Vector2d measurement;
    measurement(0) = floor_pose.translation()(0) -
                     y_infinite_room.second.node->estimate().translation()(0);
    measurement(1) = floor_pose.translation()(1) -
                     y_infinite_room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor_node, y_infinite_room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }
}

void FloorMapper::remove_floor_room_nodes(std::shared_ptr<GraphSLAM>& graph_slam,
                                          g2o::VertexFloor* floor_node) {
  std::set<g2o::HyperGraph::Edge*> floor_edges = floor_node->edges();
  for (auto edge_itr = floor_edges.begin(); edge_itr != floor_edges.end(); ++edge_itr) {
    g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(*edge_itr);
    if (edge_floor_room) {
      g2o::VertexFloor* found_room_node =
          dynamic_cast<g2o::VertexFloor*>(edge_floor_room->vertices()[1]);
      graph_slam->remove_room_room_edge(edge_floor_room);
    }
  }
}

}  // namespace s_graphs
