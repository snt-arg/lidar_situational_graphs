// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/floor_mapper.hpp>

namespace s_graphs {

FloorMapper::FloorMapper(const ros::NodeHandle& private_nh) {}

FloorMapper::~FloorMapper() {}

void FloorMapper::factor_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, const std::vector<s_graphs::floors>& floor_vec) {
  g2o::VertexRoomXYLB* floor_node;
  Eigen::Vector2d floor_pose(room_data.room_center.x, room_data.room_center.y);

  floor_node = graph_slam->add_room_node(floor_pose);
}

}  // namespace s_graphs