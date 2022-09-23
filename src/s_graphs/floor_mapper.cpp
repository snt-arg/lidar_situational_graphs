// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/floor_mapper.hpp>

namespace s_graphs {

FloorMapper::FloorMapper(const ros::NodeHandle& private_nh) {
  nh = private_nh;
}

FloorMapper::~FloorMapper() {}

void FloorMapper::lookup_floors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec) {
  double floor_threshold = 0.5;

  if(floors_vec.empty()) factor_floor_node(graph_slam, room_data, floors_vec);

  for(const auto& floor : floors_vec) {
    if(floor.id == room_data.id) {
      double floor_dist = sqrt(pow(floor.node->estimate()(0) - room_data.room_center.x, 2) + pow(floor.node->estimate()(1) - room_data.room_center.y, 2));
      if(floor_dist > floor_threshold) {
        update_floor_node(graph_slam, floor.node, room_data);
      }
    } else {
      factor_floor_node(graph_slam, room_data, floors_vec);
    }
  }
}

void FloorMapper::factor_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec) {
  g2o::VertexRoomXYLB* floor_node;
  Eigen::Vector2d floor_pose(room_data.room_center.x, room_data.room_center.y);

  Floors det_floor;
  det_floor.graph_id = graph_slam->num_vertices_local();
  floor_node = graph_slam->add_floor_node(floor_pose);
  det_floor.id = room_data.id;
  det_floor.plane_x1_id = room_data.x_planes[0].id;
  det_floor.plane_x2_id = room_data.x_planes[1].id;
  det_floor.plane_y1_id = room_data.y_planes[0].id;
  det_floor.plane_y2_id = room_data.y_planes[1].id;
  det_floor.node = floor_node;
  floors_vec.push_back(det_floor);
}

void FloorMapper::update_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node, const s_graphs::RoomData room_data) {
  Eigen::Vector2d floor_pose(room_data.room_center.x, room_data.room_center.y);
  graph_slam->update_floor_node(floor_node, floor_pose);
}

}  // namespace s_graphs