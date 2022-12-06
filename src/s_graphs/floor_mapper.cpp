// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/floor_mapper.hpp>

namespace s_graphs {

FloorMapper::FloorMapper(const ros::NodeHandle& private_nh) {
  nh = private_nh;
}

FloorMapper::~FloorMapper() {}

void FloorMapper::lookup_floors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::Corridors>& x_corridors, const std::vector<s_graphs::Corridors>& y_corridors) {
  double floor_threshold = 0.5;

  if(floors_vec.empty()) factor_floor_node(graph_slam, room_data, floors_vec, rooms_vec, x_corridors, y_corridors);

  for(const auto& floor : floors_vec) {
    if(floor.id == room_data.id) {
      double floor_dist = sqrt(pow(floor.node->estimate()(0) - room_data.room_center.x, 2) + pow(floor.node->estimate()(1) - room_data.room_center.y, 2));
      if(floor_dist > floor_threshold) {
        update_floor_node(graph_slam, floor.node, room_data, rooms_vec, x_corridors, y_corridors);
      }
    } else {
      factor_floor_node(graph_slam, room_data, floors_vec, rooms_vec, x_corridors, y_corridors);
    }
  }
}

void FloorMapper::factor_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::Corridors>& x_corridors, const std::vector<s_graphs::Corridors>& y_corridors) {
  g2o::VertexRoomXYLB* floor_node;
  Eigen::Vector2d floor_pose(room_data.room_center.x, room_data.room_center.y);

  Floors det_floor;
  det_floor.graph_id = graph_slam->retrieve_local_nbr_of_vertices();
  floor_node = graph_slam->add_floor_node(floor_pose);
  det_floor.id = room_data.id;
  det_floor.plane_x1_id = room_data.x_planes[0].id;
  det_floor.plane_x2_id = room_data.x_planes[1].id;
  det_floor.plane_y1_id = room_data.y_planes[0].id;
  det_floor.plane_y2_id = room_data.y_planes[1].id;
  det_floor.node = floor_node;
  floors_vec.push_back(det_floor);

  factor_floor_room_nodes(graph_slam, floor_pose, floor_node, rooms_vec, x_corridors, y_corridors);
}

void FloorMapper::update_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node, const s_graphs::RoomData room_data, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::Corridors>& x_corridors, const std::vector<s_graphs::Corridors>& y_corridors) {
  Eigen::Vector2d floor_pose(room_data.room_center.x, room_data.room_center.y);
  graph_slam->update_floor_node(floor_node, floor_pose);
  factor_floor_room_nodes(graph_slam, floor_pose, floor_node, rooms_vec, x_corridors, y_corridors);
}

void FloorMapper::factor_floor_room_nodes(std::unique_ptr<GraphSLAM>& graph_slam, const Eigen::Vector2d& floor_pose, g2o::VertexRoomXYLB* floor_node, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::Corridors>& x_corridors, const std::vector<s_graphs::Corridors>& y_corridors) {
  Eigen::Matrix2d information_floor;
  information_floor(0, 0) = 0.0001;
  information_floor(1, 1) = 0.0001;

  remove_floor_room_nodes(graph_slam, floor_node);

  for(const auto& room : rooms_vec) {
    Eigen::Vector2d measurement;
    measurement(0) = floor_pose(0) - room.node->estimate()(0);
    measurement(1) = floor_pose(1) - room.node->estimate()(1);

    auto edge = graph_slam->add_room_room_edge(floor_node, room.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  for(const auto& x_corridor : x_corridors) {
    Eigen::Vector2d measurement;
    measurement(0) = floor_pose(0) - x_corridor.node->estimate()(0);
    measurement(1) = floor_pose(1) - x_corridor.node->estimate()(1);

    auto edge = graph_slam->add_room_room_edge(floor_node, x_corridor.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  for(const auto& y_corridor : y_corridors) {
    Eigen::Vector2d measurement;
    measurement(0) = floor_pose(0) - y_corridor.node->estimate()(0);
    measurement(1) = floor_pose(1) - y_corridor.node->estimate()(1);

    auto edge = graph_slam->add_room_room_edge(floor_node, y_corridor.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }
}

void FloorMapper::remove_floor_room_nodes(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node) {
  std::set<g2o::HyperGraph::Edge*> floor_edges = floor_node->edges();
  for(auto edge_itr = floor_edges.begin(); edge_itr != floor_edges.end(); ++edge_itr) {
    g2o::EdgeRoomRoom* edge_floor_room = dynamic_cast<g2o::EdgeRoomRoom*>(*edge_itr);
    if(edge_floor_room) {
      g2o::VertexRoomXYLB* found_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_floor_room->vertices()[1]);
      graph_slam->remove_room_room_edge(edge_floor_room);
    }
  }
}

}  // namespace s_graphs
