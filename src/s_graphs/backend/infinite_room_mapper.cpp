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

#include <s_graphs/backend/room_mapper.hpp>

namespace s_graphs {

InfiniteRoomMapper::InfiniteRoomMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;

  infinite_room_information = node->get_parameter("infinite_room_information")
                                  .get_parameter_value()
                                  .get<double>();
  infinite_room_dist_threshold = node->get_parameter("infinite_room_dist_threshold")
                                     .get_parameter_value()
                                     .get<double>();
  dupl_plane_matching_information =
      node->get_parameter("dupl_plane_matching_information")
          .get_parameter_value()
          .get<double>();

  use_parallel_plane_constraint = node->get_parameter("use_parallel_plane_constraint")
                                      .get_parameter_value()
                                      .get<bool>();
  use_perpendicular_plane_constraint =
      node->get_parameter("use_perpendicular_plane_constraint")
          .get_parameter_value()
          .get<bool>();
}

InfiniteRoomMapper::~InfiniteRoomMapper() {}

bool InfiniteRoomMapper::lookup_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int& plane_type,
    const s_graphs::msg::RoomData room_data,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    const std::unordered_map<int, Rooms>& rooms_vec) {
  bool same_floor_level;
  std::unordered_map<int, s_graphs::VerticalPlanes>::const_iterator found_plane1;
  std::unordered_map<int, s_graphs::VerticalPlanes>::const_iterator found_plane2;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    found_plane1 = x_vert_planes.find(room_data.x_planes[0].id);
    found_plane2 = x_vert_planes.find(room_data.x_planes[1].id);
    same_floor_level =
        (found_plane1->second.floor_level == found_plane2->second.floor_level);
  } else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    found_plane1 = y_vert_planes.find(room_data.y_planes[0].id);
    found_plane2 = y_vert_planes.find(room_data.y_planes[1].id);
    same_floor_level =
        (found_plane1->second.floor_level == found_plane2->second.floor_level);
  }

  if (!same_floor_level) return false;

  int current_floor_level = found_plane1->second.floor_level;
  Eigen::Isometry3d room_center;
  Eigen::Quaterniond room_quat;
  bool duplicate_found = false;

  room_quat.x() = room_data.room_center.orientation.x;
  room_quat.y() = room_data.room_center.orientation.y;
  room_quat.z() = room_data.room_center.orientation.z;
  room_quat.w() = room_data.room_center.orientation.w;
  room_center.linear() = room_quat.toRotationMatrix();
  room_center.translation().x() = room_data.room_center.position.x;
  room_center.translation().y() = room_data.room_center.position.y;
  room_center.translation().z() = room_data.room_center.position.z;

  Eigen::Isometry3d cluster_center;
  Eigen::Quaterniond cluster_quat;
  cluster_quat.x() = 0;
  cluster_quat.y() = 0;
  cluster_quat.z() = 0;
  cluster_quat.w() = 1;
  cluster_center.linear() = cluster_quat.toRotationMatrix();
  cluster_center.translation().x() = room_data.cluster_center.x;
  cluster_center.translation().y() = room_data.cluster_center.y;
  cluster_center.translation().z() = room_data.cluster_center.z;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    // check the distance with the current room vector
    Rooms matched_room;
    float min_dist_x_inf_room_room = 100;
    for (const auto& current_room : rooms_vec) {
      if ((room_data.x_planes[0].id == current_room.second.plane_x1_id ||
           room_data.x_planes[0].id == current_room.second.plane_x2_id) &&
          (room_data.x_planes[1].id == current_room.second.plane_x1_id ||
           room_data.x_planes[1].id == current_room.second.plane_x2_id)) {
        min_dist_x_inf_room_room = 0;
        matched_room = current_room.second;
        break;
      }

      if (current_room.second.floor_level != current_floor_level) continue;

      float dist_x_inf_room_room =
          sqrt(pow(room_data.room_center.position.x -
                       current_room.second.node->estimate().translation()(0),
                   2) +
               pow(room_data.room_center.position.y -
                       current_room.second.node->estimate().translation()(1),
                   2));
      if (dist_x_inf_room_room < min_dist_x_inf_room_room) {
        min_dist_x_inf_room_room = dist_x_inf_room_room;
        matched_room = current_room.second;
      }
    }

    if (min_dist_x_inf_room_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting an x "
                   "infinite_room"
                << std::endl;
      return duplicate_found;
    }

    // factor the infinite_room here
    std::cout << "factoring x infinite_room" << std::endl;

    Eigen::Vector4d x_plane1(room_data.x_planes[0].nx,
                             room_data.x_planes[0].ny,
                             room_data.x_planes[0].nz,
                             room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(room_data.x_planes[1].nx,
                             room_data.x_planes[1].ny,
                             room_data.x_planes[1].nz,
                             room_data.x_planes[1].d);
    plane_data_list x_plane1_data, x_plane2_data;
    x_plane1_data.plane_id = room_data.x_planes[0].id;
    x_plane1_data.plane_unflipped = x_plane1;
    x_plane2_data.plane_id = room_data.x_planes[1].id;
    x_plane2_data.plane_unflipped = x_plane2;

    duplicate_found = factor_infinite_rooms(graph_slam,
                                            PlaneUtils::plane_class::X_VERT_PLANE,
                                            x_plane1_data,
                                            x_plane2_data,
                                            x_vert_planes,
                                            y_vert_planes,
                                            dupl_x_vert_planes,
                                            dupl_y_vert_planes,
                                            x_infinite_rooms,
                                            y_infinite_rooms,
                                            room_center,
                                            cluster_center,
                                            room_data.cluster_array);
  }

  else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    float min_dist_y_inf_room_room = 100;
    Rooms matched_room;
    for (const auto& current_room : rooms_vec) {
      if ((room_data.y_planes[0].id == current_room.second.plane_y1_id ||
           room_data.y_planes[0].id == current_room.second.plane_y2_id) &&
          (room_data.y_planes[1].id == current_room.second.plane_y1_id ||
           room_data.y_planes[1].id == current_room.second.plane_y2_id)) {
        min_dist_y_inf_room_room = 0;
        matched_room = current_room.second;
        break;
      }

      if (current_room.second.floor_level != current_floor_level) continue;

      float dist_y_inf_room_room =
          sqrt(pow(room_data.room_center.position.x -
                       current_room.second.node->estimate().translation()(0),
                   2) +
               pow(room_data.room_center.position.y -
                       current_room.second.node->estimate().translation()(1),
                   2));
      if (dist_y_inf_room_room < min_dist_y_inf_room_room) {
        min_dist_y_inf_room_room = dist_y_inf_room_room;
        matched_room = current_room.second;
      }
    }
    if (min_dist_y_inf_room_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting a y "
                   "infinite_room"
                << std::endl;
      return duplicate_found;
    }

    // factor the infinite_room here
    std::cout << "factoring y infinite_room" << std::endl;
    Eigen::Vector4d y_plane1(room_data.y_planes[0].nx,
                             room_data.y_planes[0].ny,
                             room_data.y_planes[0].nz,
                             room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(room_data.y_planes[1].nx,
                             room_data.y_planes[1].ny,
                             room_data.y_planes[1].nz,
                             room_data.y_planes[1].d);
    plane_data_list y_plane1_data, y_plane2_data;
    y_plane1_data.plane_id = room_data.y_planes[0].id;
    y_plane1_data.plane_unflipped = y_plane1;
    y_plane2_data.plane_id = room_data.y_planes[1].id;
    y_plane2_data.plane_unflipped = y_plane2;

    duplicate_found = factor_infinite_rooms(graph_slam,
                                            PlaneUtils::plane_class::Y_VERT_PLANE,
                                            y_plane1_data,
                                            y_plane2_data,
                                            x_vert_planes,
                                            y_vert_planes,
                                            dupl_x_vert_planes,
                                            dupl_y_vert_planes,
                                            x_infinite_rooms,
                                            y_infinite_rooms,
                                            room_center,
                                            cluster_center,
                                            room_data.cluster_array);
  }

  return duplicate_found;
}

bool InfiniteRoomMapper::factor_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int plane_type,
    const plane_data_list& room_plane1_pair,
    const plane_data_list& room_plane2_pair,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    const Eigen::Isometry3d& room_center,
    const Eigen::Isometry3d& cluster_center,
    const visualization_msgs::msg::MarkerArray& cluster_array) {
  g2o::VertexRoom* room_node;
  g2o::VertexRoom* cluster_center_node;
  int room_data_association;
  bool duplicate_found = false;

  Eigen::Matrix<double, 2, 2> information_infinite_room_planes;
  information_infinite_room_planes.setZero();
  information_infinite_room_planes(0, 0) = infinite_room_information;
  information_infinite_room_planes(1, 1) = infinite_room_information;

  Eigen::Matrix<double, 1, 1> information_infinite_room_plane;
  information_infinite_room_plane(0, 0) = infinite_room_information;

  Eigen::Matrix<double, 1, 1> information_infinite_room_prior;
  information_infinite_room_prior(0, 0) = 1e-5;

  Eigen::Matrix<double, 3, 3> information_2planes;
  information_2planes.setZero();
  information_2planes(0, 0) = dupl_plane_matching_information;
  information_2planes(1, 1) = dupl_plane_matching_information;
  information_2planes(2, 2) = dupl_plane_matching_information;

  RCLCPP_DEBUG(node_obj->get_logger(),
               "infinite_room planes",
               "final infinite_room plane 1 %f %f %f %f",
               room_plane1_pair.plane_unflipped.coeffs()(0),
               room_plane1_pair.plane_unflipped.coeffs()(1),
               room_plane1_pair.plane_unflipped.coeffs()(2),
               room_plane1_pair.plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(),
               "infinite_room planes",
               "final infinite_room plane 2 %f %f %f %f",
               room_plane2_pair.plane_unflipped.coeffs()(0),
               room_plane2_pair.plane_unflipped.coeffs()(1),
               room_plane2_pair.plane_unflipped.coeffs()(2),
               room_plane2_pair.plane_unflipped.coeffs()(3));

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    auto found_plane1 = x_vert_planes.find(room_plane1_pair.plane_id);
    auto found_plane2 = x_vert_planes.find(room_plane2_pair.plane_id);

    if (found_plane1 == x_vert_planes.end() || found_plane2 == x_vert_planes.end()) {
      std::cout << "did not find planes for x infinite_room " << std::endl;
      return duplicate_found;
    }

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    room_data_association = associate_infinite_rooms(plane_type,
                                                     room_center,
                                                     (found_plane1->second),
                                                     (found_plane2->second),
                                                     x_vert_planes,
                                                     y_vert_planes,
                                                     x_infinite_rooms,
                                                     y_infinite_rooms,
                                                     detected_mapped_plane_pairs);

    if ((x_infinite_rooms.empty() || room_data_association == -1)) {
      std::cout << "found an X infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center);
      cluster_center_node = graph_slam->add_room_node(cluster_center);
      cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_array = cluster_array;

      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.plane1_node = (found_plane1->second).plane_node;
      det_infinite_room.plane2_node = (found_plane2->second).plane_node;
      det_infinite_room.local_graph = std::make_shared<GraphSLAM>();
      det_infinite_room.floor_level = (found_plane1->second).floor_level;
      x_infinite_rooms.insert({det_infinite_room.id, det_infinite_room});

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (found_plane1->second).plane_node,
                                            (found_plane2->second).plane_node,
                                            cluster_center_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);

    } else {
      /* add the edge between detected planes and the infinite_room */
      room_node = x_infinite_rooms[room_data_association].node;
      std::cout << "Matched det infinite_room X with pre pose "
                << room_center.translation() << " to mapped infinite_room with id "
                << room_data_association << " and pose "
                << room_node->estimate().translation() << std::endl;
      x_infinite_rooms[room_data_association].cluster_array = cluster_array;

      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (found_plane1->second).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (found_plane2->second).plane_node->edges();

      if (detected_mapped_plane_pairs[0].first.id !=
          detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new x1 plane " << std::endl;
        }
      }
      if (detected_mapped_plane_pairs[1].first.id !=
          detected_mapped_plane_pairs[1].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[1].first.plane_node,
              detected_mapped_plane_pairs[1].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new x2 plane " << std::endl;
        }
      }
    }
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 = y_vert_planes.find(room_plane1_pair.plane_id);
    auto found_plane2 = y_vert_planes.find(room_plane2_pair.plane_id);

    if (found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end())
      return duplicate_found;

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    room_data_association = associate_infinite_rooms(plane_type,
                                                     room_center,
                                                     (found_plane1->second),
                                                     (found_plane2->second),
                                                     x_vert_planes,
                                                     y_vert_planes,
                                                     x_infinite_rooms,
                                                     y_infinite_rooms,
                                                     detected_mapped_plane_pairs);

    if ((y_infinite_rooms.empty() || room_data_association == -1)) {
      std::cout << "found an Y infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center);
      cluster_center_node = graph_slam->add_room_node(cluster_center);
      cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_array = cluster_array;

      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.plane1_node = (found_plane1->second).plane_node;
      det_infinite_room.plane2_node = (found_plane2->second).plane_node;
      det_infinite_room.local_graph = std::make_shared<GraphSLAM>();
      det_infinite_room.floor_level = (found_plane1->second).floor_level;
      y_infinite_rooms.insert({det_infinite_room.id, det_infinite_room});

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (found_plane1->second).plane_node,
                                            (found_plane2->second).plane_node,
                                            cluster_center_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);
    } else {
      /* add the edge between detected planes and the infinite_room */
      room_node = y_infinite_rooms[room_data_association].node;
      std::cout << "Matched det infinite_room Y with pre pose "
                << room_center.translation() << " to mapped infinite_room with id "
                << room_data_association << " and pose "
                << room_node->estimate().translation() << std::endl;
      y_infinite_rooms[room_data_association].cluster_array = cluster_array;

      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (found_plane1->second).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (found_plane2->second).plane_node->edges();

      if (detected_mapped_plane_pairs[0].first.id !=
          detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new y1 plane " << std::endl;
        }
      }
      if (detected_mapped_plane_pairs[1].first.id !=
          detected_mapped_plane_pairs[1].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[1].first.plane_node,
              detected_mapped_plane_pairs[1].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          duplicate_found = true;
          std::cout << "Adding new y2 plane " << std::endl;
        }
      }
    }
  }

  return duplicate_found;
}

int InfiniteRoomMapper::associate_infinite_rooms(
    const int& plane_type,
    const Eigen::Isometry3d& room_center,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) {
  int current_floor_level = plane1.floor_level;
  float min_dist = 100;
  bool plane1_min_segment = false, plane2_min_segment = false;

  int data_association;
  data_association = -1;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for (const auto& x_inf_room : x_infinite_rooms) {
      if (x_inf_room.second.floor_level != current_floor_level) continue;
      float dist = sqrt(pow(room_center.translation()(0) -
                                x_inf_room.second.node->estimate().translation()(0),
                            2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;

      auto found_mapped_plane1 = x_vert_planes.find(x_inf_room.second.plane1_id);
      auto found_mapped_plane2 = x_vert_planes.find(x_inf_room.second.plane2_id);

      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map) &&
            found_mapped_plane1->second.floor_level == plane1.floor_level;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map) &&
            found_mapped_plane2->second.floor_level == plane1.floor_level;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map) &&
            found_mapped_plane1->second.floor_level == plane2.floor_level;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map) &&
            found_mapped_plane2->second.floor_level == plane2.floor_level;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

      if (dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association = x_inf_room.second.id;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist x room %f", dist);
      }
    }
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for (const auto& y_inf_room : y_infinite_rooms) {
      if (y_inf_room.second.floor_level != current_floor_level) continue;
      float dist = sqrt(pow(room_center.translation()(1) -
                                y_inf_room.second.node->estimate().translation()(1),
                            2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = y_vert_planes.find(y_inf_room.second.plane1_id);
      auto found_mapped_plane2 = y_vert_planes.find(y_inf_room.second.plane2_id);

      if (plane1.id == (found_mapped_plane1->second).id ||
          plane1.id == (found_mapped_plane2->second).id) {
        plane1_min_segment = true;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane1_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane1->second).cloud_seg_map, plane1.cloud_seg_map) &&
            found_mapped_plane1->second.floor_level == plane1.floor_level;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane1_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane2->second).cloud_seg_map, plane1.cloud_seg_map) &&
            found_mapped_plane2->second.floor_level == plane1.floor_level;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

      if (plane2.id == (found_mapped_plane1->second).id ||
          plane2.id == (found_mapped_plane2->second).id) {
        plane2_min_segment = true;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (found_mapped_plane1->second)
                         .plane_node->estimate()
                         .coeffs()
                         .head(3)) > 0) {
        plane2_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane1->second).cloud_seg_map, plane2.cloud_seg_map) &&
            found_mapped_plane1->second.floor_level == plane2.floor_level;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
      } else {
        plane2_min_segment =
            PlaneUtils::check_point_neighbours(
                (found_mapped_plane2->second).cloud_seg_map, plane2.cloud_seg_map) &&
            found_mapped_plane2->second.floor_level == plane2.floor_level;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
      }
      current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

      if (dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association = y_inf_room.second.id;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist y room %f", dist);
      }
    }
  }

  // RCLCPP_DEBUG(node_obj->get_logger(),"infinite_room planes", "min dist %f",
  // min_dist);
  if (min_dist > infinite_room_dist_threshold) data_association = -1;

  return data_association;
}

bool InfiniteRoomMapper::check_infinite_room_ids(
    const int plane_type,
    const std::set<g2o::HyperGraph::Edge*>& plane_edges,
    const g2o::VertexRoom* room_node) {
  for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoom* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }

    if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoom* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }
  }
  return false;
}

}  // namespace s_graphs
