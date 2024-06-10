#include <s_graphs/backend/wall_mapper.hpp>

namespace s_graphs {

WallMapper::WallMapper(const rclcpp::Node::SharedPtr node) { node_obj = node; }

WallMapper::~WallMapper() {}

void WallMapper::factor_wall(
    const std::shared_ptr<GraphSLAM> covisibility_graph,
    const Eigen::Vector3d& wall_pose,
    const Eigen::Vector3d& wall_point,
    const std::vector<situational_graphs_msgs::msg::PlaneData> x_planes_msg,
    const std::vector<situational_graphs_msgs::msg::PlaneData> y_planes_msg,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes) {
  if (x_planes_msg.size() == 2) {
    auto matched_x_plane1 = x_vert_planes.find(x_planes_msg[0].id);
    auto matched_x_plane2 = x_vert_planes.find(x_planes_msg[1].id);

    bool same_floor_level =
        (matched_x_plane1->second.floor_level == matched_x_plane2->second.floor_level);

    if (!same_floor_level) return;

    if (!(matched_x_plane1->second).on_wall && !(matched_x_plane2->second).on_wall) {
      add_wall_node_and_edge(covisibility_graph,
                             wall_pose,
                             wall_point,
                             matched_x_plane1->second,
                             matched_x_plane2->second);
    }
  } else if (y_planes_msg.size() == 2) {
    auto matched_y_plane1 = y_vert_planes.find(y_planes_msg[0].id);
    auto matched_y_plane2 = y_vert_planes.find(y_planes_msg[1].id);

    bool same_floor_level =
        (matched_y_plane1->second.floor_level == matched_y_plane2->second.floor_level);

    if (!same_floor_level) return;

    if (!(matched_y_plane1->second).on_wall && !(matched_y_plane2->second).on_wall) {
      add_wall_node_and_edge(covisibility_graph,
                             wall_pose,
                             wall_point,
                             matched_y_plane1->second,
                             matched_y_plane2->second);
    }
  } else {
    std::cout << "Message size is not 2 !! " << std::endl;
  }
}

void WallMapper::add_wall_node_and_edge(
    const std::shared_ptr<GraphSLAM> covisibility_graph,
    const Eigen::Vector3d& wall_pose,
    const Eigen::Vector3d& wall_point,
    VerticalPlanes& plane1,
    VerticalPlanes& plane2) {
  g2o::VertexWall* wall_node = covisibility_graph->add_wall_node(wall_pose);
  Eigen::Matrix<double, 3, 3> information_wall_surfaces;
  information_wall_surfaces.setZero();
  information_wall_surfaces(0, 0) = 10;
  information_wall_surfaces(1, 1) = 10;
  information_wall_surfaces(2, 2) = 10;
  auto wall_edge = covisibility_graph->add_wall_2planes_edge(wall_node,
                                                             (plane1).plane_node,
                                                             (plane2).plane_node,
                                                             wall_point,
                                                             information_wall_surfaces);
  covisibility_graph->add_robust_kernel(wall_edge, "Huber", 1.0);
  (plane1).on_wall = true;
  (plane2).on_wall = true;
}
}  // namespace s_graphs