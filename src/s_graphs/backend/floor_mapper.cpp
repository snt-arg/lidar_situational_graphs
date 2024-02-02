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
  floor_horizontal_threshold = 0.5;
  floor_vertical_threshold = 1.0;
  floor_level_updated = false;
}

FloorMapper::~FloorMapper() {}

void FloorMapper::lookup_floors(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const s_graphs::msg::RoomData room_data,
    std::unordered_map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Vector3d floor_center(room_data.room_center.position.x,
                               room_data.room_center.position.y,
                               room_data.room_center.position.z);
  int data_association = associate_floors(floor_center, floors_vec);
  if (floors_vec.empty()) {
    int floor_id = factor_floor_node(graph_slam,
                                     room_data,
                                     floors_vec,
                                     rooms_vec,
                                     x_infinite_rooms,
                                     y_infinite_rooms);
    set_floor_level(floor_id);
  } else if (data_association == -1) {
    remove_floor_room_nodes(graph_slam, floors_vec);
    int floor_id = factor_floor_node(graph_slam,
                                     room_data,
                                     floors_vec,
                                     rooms_vec,
                                     x_infinite_rooms,
                                     y_infinite_rooms);
    set_floor_level(floor_id);
    update_floor_level(true);
  } else if (data_association != -1) {
    double floor_dist =
        sqrt(pow(floors_vec[data_association].node->estimate().translation()(0) -
                     room_data.room_center.position.x,
                 2) +
             pow(floors_vec[data_association].node->estimate().translation()(1) -
                     room_data.room_center.position.y,
                 2));
    if (floor_dist > floor_horizontal_threshold) {
      remove_floor_room_nodes(graph_slam, floors_vec);
      update_floor_node(graph_slam,
                        floors_vec[data_association].node,
                        room_data,
                        floors_vec,
                        rooms_vec,
                        x_infinite_rooms,
                        y_infinite_rooms);
      set_floor_level(data_association);
    }
  }
}

int FloorMapper::associate_floors(const Eigen::Vector3d& floor_center,
                                  const std::unordered_map<int, Floors>& floors_vec) {
  double min_z_dist = 100;
  int data_association = -1;
  // just check the just of the floors
  for (const auto& mapped_floor : floors_vec) {
    double z_dist = floor_center(2) - mapped_floor.second.node->estimate()(2, 3);

    if (z_dist < min_z_dist) {
      min_z_dist = z_dist;
      data_association = mapped_floor.first;
    }
  }

  if (min_z_dist > floor_vertical_threshold) data_association = -1;

  return data_association;
}

int FloorMapper::factor_floor_node(
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
  det_floor.id = det_floor.graph_id;
  if (!room_data.x_planes.empty() && !room_data.y_planes.empty()) {
    det_floor.plane_x1_id = room_data.x_planes[0].id;
    det_floor.plane_x2_id = room_data.x_planes[1].id;
    det_floor.plane_y1_id = room_data.y_planes[0].id;
    det_floor.plane_y2_id = room_data.y_planes[1].id;
  }
  det_floor.node = floor_node;
  if (floors_vec.empty()) {
    det_floor.color.push_back(0);
    det_floor.color.push_back(0);
    det_floor.color.push_back(0);
  } else
    det_floor.color = PlaneUtils::random_color_vec();
  floors_vec.insert({det_floor.id, det_floor});

  factor_floor_room_nodes(
      graph_slam, floors_vec, rooms_vec, x_infinite_rooms, y_infinite_rooms);

  return det_floor.id;
}

void FloorMapper::update_floor_node(
    std::shared_ptr<GraphSLAM>& graph_slam,
    g2o::VertexFloor* floor_node,
    const s_graphs::msg::RoomData room_data,
    std::unordered_map<int, s_graphs::Floors>& floors_vec,
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
  factor_floor_room_nodes(
      graph_slam, floors_vec, rooms_vec, x_infinite_rooms, y_infinite_rooms);
}

void FloorMapper::factor_floor_room_nodes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    std::unordered_map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Matrix2d information_floor;
  information_floor(0, 0) = 0.0001;
  information_floor(1, 1) = 0.0001;

  for (const auto& floor : floors_vec) {
    for (const auto& room : rooms_vec) {
      if (room.second.floor_level != floor.first) continue;

      Eigen::Vector2d measurement;
      measurement(0) = floor.second.node->estimate().translation()(0) -
                       room.second.node->estimate().translation()(0);
      measurement(1) = floor.second.node->estimate().translation()(1) -
                       room.second.node->estimate().translation()(1);

      auto edge = graph_slam->add_floor_room_edge(
          floor.second.node, room.second.node, measurement, information_floor);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }

    for (const auto& x_infinite_room : x_infinite_rooms) {
      if (x_infinite_room.second.floor_level != floor.first) continue;

      Eigen::Vector2d measurement;
      measurement(0) = floor.second.node->estimate().translation()(0) -
                       x_infinite_room.second.node->estimate().translation()(0);
      measurement(1) = floor.second.node->estimate().translation()(1) -
                       x_infinite_room.second.node->estimate().translation()(1);

      auto edge = graph_slam->add_floor_room_edge(floor.second.node,
                                                  x_infinite_room.second.node,
                                                  measurement,
                                                  information_floor);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }

    for (const auto& y_infinite_room : y_infinite_rooms) {
      if (y_infinite_room.second.floor_level != floor.first) continue;

      Eigen::Vector2d measurement;
      measurement(0) = floor.second.node->estimate().translation()(0) -
                       y_infinite_room.second.node->estimate().translation()(0);
      measurement(1) = floor.second.node->estimate().translation()(1) -
                       y_infinite_room.second.node->estimate().translation()(1);

      auto edge = graph_slam->add_floor_room_edge(floor.second.node,
                                                  y_infinite_room.second.node,
                                                  measurement,
                                                  information_floor);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }
  }
}

void FloorMapper::remove_floor_room_nodes(std::shared_ptr<GraphSLAM>& graph_slam,
                                          std::unordered_map<int, Floors>& floors_vec) {
  std::vector<g2o::EdgeFloorRoom*> edge_floor_room_vec;
  for (const auto& floor : floors_vec) {
    g2o::VertexFloor* floor_node = floor.second.node;
    for (g2o::HyperGraph::EdgeSet::iterator e_it = floor_node->edges().begin();
         e_it != floor_node->edges().end();
         ++e_it) {
      g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*e_it);
      g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(e);
      if (edge_floor_room != nullptr) {
        edge_floor_room_vec.push_back(edge_floor_room);
      }
    }
  }

  for (auto edge_floor_room : edge_floor_room_vec) {
    bool ack = graph_slam->remove_room_room_edge(edge_floor_room);
  }
}

}  // namespace s_graphs
