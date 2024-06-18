#include <s_graphs/backend/floor_mapper.hpp>

namespace s_graphs {

FloorMapper::FloorMapper(std::mutex& graph_mutex) : shared_graph_mutex(graph_mutex) {
  floor_horizontal_threshold = 0.5;
  floor_vertical_threshold = 0.5;
  floor_level_updated = false;
}

FloorMapper::~FloorMapper() {}

void FloorMapper::lookup_floors(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::FloorData floor_data,
    std::map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Vector3d floor_center(floor_data.floor_center.position.x,
                               floor_data.floor_center.position.y,
                               floor_data.floor_center.position.z);
  int data_association = associate_floors(floor_center, floors_vec);
  if (floors_vec.empty()) {
    int floor_id = factor_floor_node(graph_slam,
                                     floor_data,
                                     floors_vec,
                                     rooms_vec,
                                     x_infinite_rooms,
                                     y_infinite_rooms);
    set_floor_level(floor_id);
  } else if (data_association == -1) {
    int floor_id = factor_floor_node(graph_slam,
                                     floor_data,
                                     floors_vec,
                                     rooms_vec,
                                     x_infinite_rooms,
                                     y_infinite_rooms);
    set_floor_level(floor_id);
    update_floor_level(true);
  } else if (data_association != -1) {
    shared_graph_mutex.lock();
    double floor_dist =
        sqrt(pow(floors_vec[data_association].node->estimate().translation()(0) -
                     floor_data.floor_center.position.x,
                 2) +
             pow(floors_vec[data_association].node->estimate().translation()(1) -
                     floor_data.floor_center.position.y,
                 2));
    shared_graph_mutex.unlock();

    if (floor_dist > floor_horizontal_threshold) {
      update_floor_node(graph_slam,
                        floors_vec[data_association].node,
                        floor_data,
                        floors_vec,
                        rooms_vec,
                        x_infinite_rooms,
                        y_infinite_rooms);
      set_floor_level(data_association);
    }
  }
}

int FloorMapper::associate_floors(const Eigen::Vector3d& floor_center,
                                  const std::map<int, Floors>& floors_vec) {
  double min_z_dist = 100;
  int data_association = -1;
  // just check the just of the floors
  for (const auto& mapped_floor : floors_vec) {
    shared_graph_mutex.lock();
    double z_dist = fabs(floor_center(2) - mapped_floor.second.node->estimate()(2, 3));
    shared_graph_mutex.unlock();

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
    const situational_graphs_msgs::msg::FloorData floor_data,
    std::map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  g2o::VertexFloor* floor_node;
  Eigen::Isometry3d floor_pose;
  floor_pose.setIdentity();
  floor_pose.translation().x() = floor_data.floor_center.position.x;
  floor_pose.translation().y() = floor_data.floor_center.position.y;
  floor_pose.translation().z() = floor_data.floor_center.position.z;

  Floors det_floor;

  shared_graph_mutex.lock();
  det_floor.id = graph_slam->retrieve_local_nbr_of_vertices();
  floor_node = graph_slam->add_floor_node(floor_pose);
  shared_graph_mutex.unlock();

  det_floor.sequential_id = floors_vec.size();
  det_floor.node = floor_node;
  det_floor.color = PlaneUtils::random_color_vec();
  det_floor.floor_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

  shared_graph_mutex.lock();
  floors_vec.insert({det_floor.id, det_floor});
  shared_graph_mutex.unlock();

  factor_floor_room_nodes(
      graph_slam, floors_vec, rooms_vec, x_infinite_rooms, y_infinite_rooms);

  return det_floor.id;
}

void FloorMapper::update_floor_node(
    std::shared_ptr<GraphSLAM>& graph_slam,
    g2o::VertexFloor* floor_node,
    const situational_graphs_msgs::msg::FloorData floor_data,
    std::map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Isometry3d floor_pose;
  floor_pose.setIdentity();
  floor_pose.translation().x() = floor_data.floor_center.position.x;
  floor_pose.translation().y() = floor_data.floor_center.position.y;
  floor_pose.translation().z() = floor_data.floor_center.position.z;

  shared_graph_mutex.lock();
  graph_slam->update_floor_node(floor_node, floor_pose);
  shared_graph_mutex.unlock();

  factor_floor_room_nodes(
      graph_slam, floors_vec, rooms_vec, x_infinite_rooms, y_infinite_rooms);
}

void FloorMapper::factor_floor_room_nodes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    std::map<int, s_graphs::Floors>& floors_vec,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  remove_floor_room_nodes(graph_slam, floors_vec);
  for (const auto& floor : floors_vec) {
    this->add_floor_room_edges(
        graph_slam, floor.second, rooms_vec, x_infinite_rooms, y_infinite_rooms);
  }
}

void FloorMapper::factor_floor_room_nodes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    s_graphs::Floors floor,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  remove_nodes(graph_slam, floor);
  this->add_floor_room_edges(
      graph_slam, floor, rooms_vec, x_infinite_rooms, y_infinite_rooms);
}

void FloorMapper::add_floor_room_edges(
    std::shared_ptr<GraphSLAM>& graph_slam,
    s_graphs::Floors floor,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  Eigen::Matrix3d information_floor;
  information_floor(0, 0) = 1.0;
  information_floor(1, 1) = 1.0;
  information_floor(2, 2) = 1.0;

  for (const auto& room : rooms_vec) {
    if (room.second.floor_level != floor.id) continue;

    Eigen::Vector3d measurement;
    measurement.setZero();

    shared_graph_mutex.lock();
    measurement(0) = floor.node->estimate().translation()(0) -
                     room.second.node->estimate().translation()(0);
    measurement(1) = floor.node->estimate().translation()(1) -
                     room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor.node, room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    shared_graph_mutex.unlock();
  }

  for (const auto& x_infinite_room : x_infinite_rooms) {
    if (x_infinite_room.second.floor_level != floor.id) continue;

    Eigen::Vector3d measurement;
    measurement.setZero();

    shared_graph_mutex.lock();
    measurement(0) = floor.node->estimate().translation()(0) -
                     x_infinite_room.second.node->estimate().translation()(0);
    measurement(1) = floor.node->estimate().translation()(1) -
                     x_infinite_room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor.node, x_infinite_room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    shared_graph_mutex.unlock();
  }

  for (const auto& y_infinite_room : y_infinite_rooms) {
    if (y_infinite_room.second.floor_level != floor.id) continue;

    Eigen::Vector3d measurement;
    measurement.setZero();

    shared_graph_mutex.lock();
    measurement(0) = floor.node->estimate().translation()(0) -
                     y_infinite_room.second.node->estimate().translation()(0);
    measurement(1) = floor.node->estimate().translation()(1) -
                     y_infinite_room.second.node->estimate().translation()(1);

    auto edge = graph_slam->add_floor_room_edge(
        floor.node, y_infinite_room.second.node, measurement, information_floor);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    shared_graph_mutex.unlock();
  }
}

void FloorMapper::remove_floor_room_nodes(std::shared_ptr<GraphSLAM>& graph_slam,
                                          std::map<int, Floors>& floors_vec) {
  std::vector<g2o::EdgeFloorRoom*> edge_floor_room_vec;
  for (const auto& floor : floors_vec) {
    std::vector<g2o::EdgeFloorRoom*> current_edge_floor_room_vec =
        remove_nodes(graph_slam, floor.second);

    for (const auto& current_edge : current_edge_floor_room_vec) {
      edge_floor_room_vec.push_back(current_edge);
    }
  }

  shared_graph_mutex.lock();
  for (auto edge_floor_room : edge_floor_room_vec) {
    bool ack = graph_slam->remove_room_room_edge(edge_floor_room);
  }
  shared_graph_mutex.unlock();
}

std::vector<g2o::EdgeFloorRoom*> FloorMapper::remove_nodes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    Floors floor) {
  std::vector<g2o::EdgeFloorRoom*> current_edge_floor_room_vec;

  shared_graph_mutex.lock();
  g2o::VertexFloor* floor_node = floor.node;
  for (g2o::HyperGraph::EdgeSet::iterator e_it = floor_node->edges().begin();
       e_it != floor_node->edges().end();
       ++e_it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*e_it);
    g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(e);
    if (edge_floor_room != nullptr) {
      current_edge_floor_room_vec.push_back(edge_floor_room);
    }
  }
  shared_graph_mutex.unlock();

  return current_edge_floor_room_vec;
}

}  // namespace s_graphs
