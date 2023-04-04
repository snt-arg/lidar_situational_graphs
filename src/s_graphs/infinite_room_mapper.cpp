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

#include <s_graphs/room_mapper.hpp>

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

  plane_utils.reset(new PlaneUtils());
}

InfiniteRoomMapper::~InfiniteRoomMapper() {}

void InfiniteRoomMapper::lookup_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int& plane_type,
    const s_graphs::msg::RoomData room_data,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::vector<InfiniteRooms>& x_infinite_rooms,
    std::vector<InfiniteRooms>& y_infinite_rooms,
    const std::vector<Rooms>& rooms_vec) {
  Eigen::Isometry3d room_center;
  Eigen::Quaterniond room_quat;
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
      if ((room_data.x_planes[0].id == current_room.plane_x1_id ||
           room_data.x_planes[0].id == current_room.plane_x2_id) &&
          (room_data.x_planes[1].id == current_room.plane_x1_id ||
           room_data.x_planes[1].id == current_room.plane_x2_id)) {
        min_dist_x_inf_room_room = 0;
        matched_room = current_room;
        break;
      }

      float dist_x_inf_room_room = sqrt(
          pow(room_data.room_center.position.x - current_room.node->estimate()(0), 2) +
          pow(room_data.room_center.position.y - current_room.node->estimate()(1), 2));
      if (dist_x_inf_room_room < min_dist_x_inf_room_room) {
        min_dist_x_inf_room_room = dist_x_inf_room_room;
        matched_room = current_room;
      }
    }

    if (min_dist_x_inf_room_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting an x "
                   "infinite_room"
                << std::endl;
      return;
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

    factor_infinite_rooms(graph_slam,
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
      if ((room_data.y_planes[0].id == current_room.plane_y1_id ||
           room_data.y_planes[0].id == current_room.plane_y2_id) &&
          (room_data.y_planes[1].id == current_room.plane_y1_id ||
           room_data.y_planes[1].id == current_room.plane_y2_id)) {
        min_dist_y_inf_room_room = 0;
        matched_room = current_room;
        break;
      }

      float dist_y_inf_room_room = sqrt(
          pow(room_data.room_center.position.x - current_room.node->estimate()(0), 2) +
          pow(room_data.room_center.position.y - current_room.node->estimate()(1), 2));
      if (dist_y_inf_room_room < min_dist_y_inf_room_room) {
        min_dist_y_inf_room_room = dist_y_inf_room_room;
        matched_room = current_room;
      }
    }
    if (min_dist_y_inf_room_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting a y "
                   "infinite_room"
                << std::endl;
      return;
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

    factor_infinite_rooms(graph_slam,
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
}

void InfiniteRoomMapper::factor_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const int plane_type,
    const plane_data_list& room_plane1_pair,
    const plane_data_list& room_plane2_pair,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::vector<InfiniteRooms>& x_infinite_rooms,
    std::vector<InfiniteRooms>& y_infinite_rooms,
    const Eigen::Isometry3d& room_center,
    const Eigen::Isometry3d& cluster_center,
    const visualization_msgs::msg::MarkerArray& cluster_array) {
  g2o::VertexRoomXYLB* room_node;
  g2o::VertexRoomXYLB* cluster_center_node;
  std::pair<int, int> room_data_association;
  double meas_plane1, meas_plane2;

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
    auto found_plane1 =
        std::find_if(x_vert_planes.begin(),
                     x_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == room_plane1_pair.plane_id);
    auto found_plane2 =
        std::find_if(x_vert_planes.begin(),
                     x_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == room_plane2_pair.plane_id);

    if (found_plane1 == x_vert_planes.end() || found_plane2 == x_vert_planes.end()) {
      std::cout << "did not find planes for x infinite_room " << std::endl;
      return;
    }

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    room_data_association = associate_infinite_rooms(plane_type,
                                                     room_center,
                                                     (*found_plane1),
                                                     (*found_plane2),
                                                     x_vert_planes,
                                                     y_vert_planes,
                                                     x_infinite_rooms,
                                                     y_infinite_rooms,
                                                     detected_mapped_plane_pairs);

    if ((x_infinite_rooms.empty() || room_data_association.first == -1)) {
      std::cout << "found an X infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center.translation());
      cluster_center_node = graph_slam->add_room_node(cluster_center.translation());
      cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association.first;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.cluster_array = cluster_array;
      x_infinite_rooms.push_back(det_infinite_room);

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (*found_plane1).plane_node,
                                            (*found_plane2).plane_node,
                                            cluster_center_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);

    } else {
      /* add the edge between detected planes and the infinite_room */
      room_node = x_infinite_rooms[room_data_association.second].node;
      std::cout << "Matched det infinite_room X with pre pose "
                << room_center.translation() << " to mapped infinite_room with id "
                << room_data_association.first << " and pose " << room_node->estimate()
                << std::endl;
      x_infinite_rooms[room_data_association.second].cluster_array = cluster_array;

      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (*found_plane1).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (*found_plane2).plane_node->edges();

      if (detected_mapped_plane_pairs[0].first.id !=
          detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
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
          std::cout << "Adding new x2 plane " << std::endl;
        }
      }
    }
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 =
        std::find_if(y_vert_planes.begin(),
                     y_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == room_plane1_pair.plane_id);
    auto found_plane2 =
        std::find_if(y_vert_planes.begin(),
                     y_vert_planes.end(),
                     boost::bind(&VerticalPlanes::id, _1) == room_plane2_pair.plane_id);

    if (found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end())
      return;

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    room_data_association = associate_infinite_rooms(plane_type,
                                                     room_center,
                                                     (*found_plane1),
                                                     (*found_plane2),
                                                     x_vert_planes,
                                                     y_vert_planes,
                                                     x_infinite_rooms,
                                                     y_infinite_rooms,
                                                     detected_mapped_plane_pairs);

    if ((y_infinite_rooms.empty() || room_data_association.first == -1)) {
      std::cout << "found an Y infinite_room with pre pose "
                << room_center.translation() << " between plane "
                << room_plane1_pair.plane_unflipped.coeffs() << " and plane "
                << room_plane2_pair.plane_unflipped.coeffs() << std::endl;

      room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      room_node = graph_slam->add_room_node(room_center.translation());
      cluster_center_node = graph_slam->add_room_node(cluster_center.translation());
      cluster_center_node->setFixed(true);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = room_data_association.first;
      det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = room_plane1_pair.plane_id;
      det_infinite_room.plane2_id = room_plane2_pair.plane_id;
      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = room_node;
      det_infinite_room.cluster_array = cluster_array;
      y_infinite_rooms.push_back(det_infinite_room);

      auto edge_room_plane =
          graph_slam->add_room_2planes_edge(room_node,
                                            (*found_plane1).plane_node,
                                            (*found_plane2).plane_node,
                                            cluster_center_node,
                                            information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);
    } else {
      /* add the edge between detected planes and the infinite_room */
      room_node = y_infinite_rooms[room_data_association.second].node;
      std::cout << "Matched det infinite_room Y with pre pose "
                << room_center.translation() << " to mapped infinite_room with id "
                << room_data_association.first << " and pose " << room_node->estimate()
                << std::endl;
      y_infinite_rooms[room_data_association.second].cluster_array = cluster_array;

      std::set<g2o::HyperGraph::Edge*> plane1_edges =
          (*found_plane1).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges =
          (*found_plane2).plane_node->edges();

      if (detected_mapped_plane_pairs[0].first.id !=
          detected_mapped_plane_pairs[0].second.id) {
        if (!MapperUtils::check_plane_ids(
                plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(
              detected_mapped_plane_pairs[0].first.plane_node,
              detected_mapped_plane_pairs[0].second.plane_node,
              information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
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
          std::cout << "Adding new y2 plane " << std::endl;
        }
      }
    }
  }

  return;
}

std::pair<int, int> InfiniteRoomMapper::associate_infinite_rooms(
    const int& plane_type,
    const Eigen::Isometry3d& room_center,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    const std::vector<InfiniteRooms>& x_infinite_rooms,
    const std::vector<InfiniteRooms>& y_infinite_rooms,
    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) {
  float min_dist = 100;
  bool plane1_min_segment = false, plane2_min_segment = false;

  std::pair<int, int> data_association;
  data_association.first = -1;

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for (int i = 0; i < x_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(
          room_center.translation()(0) - x_infinite_rooms[i].node->estimate()(0), 2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = std::find_if(
          x_vert_planes.begin(),
          x_vert_planes.end(),
          boost::bind(&VerticalPlanes::id, _1) == x_infinite_rooms[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(
          x_vert_planes.begin(),
          x_vert_planes.end(),
          boost::bind(&VerticalPlanes::id, _1) == x_infinite_rooms[i].plane2_id);

      if (plane1.id == (*found_mapped_plane1).id ||
          plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) >
                 0) {
        plane1_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane1_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

      if (plane2.id == (*found_mapped_plane1).id ||
          plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) >
                 0) {
        plane2_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane2_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

      if (dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = x_infinite_rooms[i].id;
        data_association.second = i;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist x room %f", dist);
      }
    }
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for (int i = 0; i < y_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(
          room_center.translation()(1) - y_infinite_rooms[i].node->estimate()(1), 2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = std::find_if(
          y_vert_planes.begin(),
          y_vert_planes.end(),
          boost::bind(&VerticalPlanes::id, _1) == y_infinite_rooms[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(
          y_vert_planes.begin(),
          y_vert_planes.end(),
          boost::bind(&VerticalPlanes::id, _1) == y_infinite_rooms[i].plane2_id);

      if (plane1.id == (*found_mapped_plane1).id ||
          plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = plane1;
      } else if ((plane1).plane_node->estimate().coeffs().head(3).dot(
                     (*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) >
                 0) {
        plane1_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane1_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

      if (plane2.id == (*found_mapped_plane1).id ||
          plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = plane2;
      } else if ((plane2).plane_node->estimate().coeffs().head(3).dot(
                     (*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) >
                 0) {
        plane2_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane2_min_segment = plane_utils->check_point_neighbours(
            (*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

      if (dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = y_infinite_rooms[i].id;
        data_association.second = i;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(
            node_obj->get_logger(), "infinite_room planes", "dist y room %f", dist);
      }
    }
  }

  // RCLCPP_DEBUG(node_obj->get_logger(),"infinite_room planes", "min dist %f",
  // min_dist);
  if (min_dist > infinite_room_dist_threshold) data_association.first = -1;

  return data_association;
}

bool InfiniteRoomMapper::check_infinite_room_ids(
    const int plane_type,
    const std::set<g2o::HyperGraph::Edge*>& plane_edges,
    const g2o::VertexRoomXYLB* room_node) {
  for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoomXYLB* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoomXYLB*>(
                edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }

    if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes =
          dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if (edge_infinite_room_planes) {
        g2o::VertexRoomXYLB* found_infinite_room_node =
            dynamic_cast<g2o::VertexRoomXYLB*>(
                edge_infinite_room_planes->vertices()[0]);
        if (found_infinite_room_node->id() == room_node->id()) return true;
      }
    }
  }
  return false;
}

}  // namespace s_graphs
