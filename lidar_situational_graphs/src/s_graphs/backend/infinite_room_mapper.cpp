#include <s_graphs/backend/room_mapper.hpp>

namespace s_graphs {

InfiniteRoomMapper::InfiniteRoomMapper(const rclcpp::Node::SharedPtr node,
                                       std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
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
    const situational_graphs_msgs::msg::RoomData room_data,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    const std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id) {
  room_id = -1;
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

  if (!same_floor_level) {
    return false;
  };

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

      shared_graph_mutex.lock();
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
      shared_graph_mutex.unlock();
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
                                            room_id);
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

      shared_graph_mutex.lock();
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
      shared_graph_mutex.unlock();
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
                                            room_id);
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
    int& room_id) {
  room_id = -1;
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

    duplicate_found = insert_infinite_room(graph_slam,
                                           room_data_association,
                                           room_id,
                                           room_center,
                                           cluster_center,
                                           found_plane1->second,
                                           found_plane2->second,
                                           room_plane1_pair,
                                           room_plane2_pair,
                                           x_infinite_rooms,
                                           detected_mapped_plane_pairs);
  }

  if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 = y_vert_planes.find(room_plane1_pair.plane_id);
    auto found_plane2 = y_vert_planes.find(room_plane2_pair.plane_id);

    if (found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end()) {
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

    duplicate_found = insert_infinite_room(graph_slam,
                                           room_data_association,
                                           room_id,
                                           room_center,
                                           cluster_center,
                                           found_plane1->second,
                                           found_plane2->second,
                                           room_plane1_pair,
                                           room_plane2_pair,
                                           y_infinite_rooms,
                                           detected_mapped_plane_pairs);
  }

  return duplicate_found;
}

bool InfiniteRoomMapper::insert_infinite_room(
    std::shared_ptr<GraphSLAM>& graph_slam,
    int room_data_association,
    int& room_id,
    const Eigen::Isometry3d& room_center,
    const Eigen::Isometry3d& cluster_center,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const plane_data_list& room_plane1_pair,
    const plane_data_list& room_plane2_pair,
    std::unordered_map<int, InfiniteRooms>& infinite_rooms,
    const std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) {
  bool duplicate_found = false;
  g2o::VertexRoom* room_node;
  g2o::VertexRoom* cluster_center_node;

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

  if ((infinite_rooms.empty() || room_data_association == -1)) {
    std::cout << "found an infinite_room with pre pose " << room_center.translation()
              << " between plane " << room_plane1_pair.plane_unflipped.coeffs()
              << " and plane " << room_plane2_pair.plane_unflipped.coeffs()
              << std::endl;

    shared_graph_mutex.lock();
    room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_center);
    cluster_center_node = graph_slam->add_room_node(cluster_center);
    cluster_center_node->setFixed(true);
    shared_graph_mutex.unlock();

    InfiniteRooms det_infinite_room;
    det_infinite_room.id = room_data_association;
    room_id = det_infinite_room.id;
    det_infinite_room.plane1 = room_plane1_pair.plane_unflipped;
    det_infinite_room.plane2 = room_plane2_pair.plane_unflipped;
    det_infinite_room.plane1_id = room_plane1_pair.plane_id;
    det_infinite_room.plane2_id = room_plane2_pair.plane_id;

    det_infinite_room.cluster_center_node = cluster_center_node;
    det_infinite_room.node = room_node;
    det_infinite_room.plane1_node = (plane1).plane_node;
    det_infinite_room.plane2_node = (plane2).plane_node;
    det_infinite_room.local_graph = std::make_shared<GraphSLAM>();
    det_infinite_room.floor_level = (plane1).floor_level;

    shared_graph_mutex.lock();
    infinite_rooms.insert({det_infinite_room.id, det_infinite_room});
    auto edge_room_plane =
        graph_slam->add_room_2planes_edge(room_node,
                                          (plane1).plane_node,
                                          (plane2).plane_node,
                                          cluster_center_node,
                                          information_infinite_room_planes);
    graph_slam->add_robust_kernel(edge_room_plane, "Huber", 1.0);
    shared_graph_mutex.unlock();
  } else {
    /* add the edge between detected planes and the infinite_room */
    shared_graph_mutex.lock();
    room_node = infinite_rooms[room_data_association].node;
    room_id = room_data_association;
    std::cout << "Matched det infinite_room with pre pose " << room_center.translation()
              << " to mapped infinite_room with id " << room_data_association
              << " and pose " << room_node->estimate().translation() << std::endl;
    shared_graph_mutex.unlock();

    shared_graph_mutex.lock();
    std::set<g2o::HyperGraph::Edge*> plane1_edges = (plane1).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> plane2_edges = (plane2).plane_node->edges();

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
        std::cout << "Adding new plane1 " << std::endl;
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
        std::cout << "Adding new plane2 " << std::endl;
      }
    }
    shared_graph_mutex.unlock();
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

      shared_graph_mutex.lock();
      float dist = sqrt(pow(room_center.translation()(0) -
                                x_inf_room.second.node->estimate().translation()(0),
                            2));
      shared_graph_mutex.unlock();

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs = match_planes(plane1_min_segment,
                                                             plane2_min_segment,
                                                             plane1,
                                                             plane2,
                                                             x_inf_room.second,
                                                             x_vert_planes);

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

      shared_graph_mutex.lock();
      float dist = sqrt(pow(room_center.translation()(1) -
                                y_inf_room.second.node->estimate().translation()(1),
                            2));
      shared_graph_mutex.unlock();

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
          current_detected_mapped_plane_pairs = match_planes(plane2_min_segment,
                                                             plane2_min_segment,
                                                             plane1,
                                                             plane2,
                                                             y_inf_room.second,
                                                             y_vert_planes);

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

std::vector<std::pair<VerticalPlanes, VerticalPlanes>> InfiniteRoomMapper::match_planes(
    bool& plane1_min_segment,
    bool& plane2_min_segment,
    const VerticalPlanes& plane1,
    const VerticalPlanes& plane2,
    const InfiniteRooms& infinite_room,
    const std::unordered_map<int, VerticalPlanes>& vert_planes) {
  std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
      current_detected_mapped_plane_pairs;
  std::pair<VerticalPlanes, VerticalPlanes> p1_detected_mapped_plane_pair;
  std::pair<VerticalPlanes, VerticalPlanes> p2_detected_mapped_plane_pair;

  auto found_mapped_plane1 = vert_planes.find(infinite_room.plane1_id);
  auto found_mapped_plane2 = vert_planes.find(infinite_room.plane2_id);

  shared_graph_mutex.lock();
  double p1_p2_product = (plane1).plane_node->estimate().coeffs().head(3).dot(
      (found_mapped_plane1->second).plane_node->estimate().coeffs().head(3));
  shared_graph_mutex.unlock();

  if (plane1.id == (found_mapped_plane1->second).id ||
      plane1.id == (found_mapped_plane2->second).id) {
    plane1_min_segment = true;
    p1_detected_mapped_plane_pair.first = plane1;
    p1_detected_mapped_plane_pair.second = plane1;
  } else if (p1_p2_product > 0) {
    plane1_min_segment =
        PlaneUtils::check_point_neighbours((found_mapped_plane1->second).cloud_seg_map,
                                           plane1.cloud_seg_map) &&
        found_mapped_plane1->second.floor_level == plane1.floor_level;
    p1_detected_mapped_plane_pair.first = plane1;
    p1_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
  } else {
    plane1_min_segment =
        PlaneUtils::check_point_neighbours((found_mapped_plane2->second).cloud_seg_map,
                                           plane1.cloud_seg_map) &&
        found_mapped_plane2->second.floor_level == plane1.floor_level;
    p1_detected_mapped_plane_pair.first = plane1;
    p1_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
  }
  current_detected_mapped_plane_pairs.push_back(p1_detected_mapped_plane_pair);

  shared_graph_mutex.lock();
  double p2_p1_product = (plane2).plane_node->estimate().coeffs().head(3).dot(
      (found_mapped_plane1->second).plane_node->estimate().coeffs().head(3));
  shared_graph_mutex.unlock();

  if (plane2.id == (found_mapped_plane1->second).id ||
      plane2.id == (found_mapped_plane2->second).id) {
    plane2_min_segment = true;
    p2_detected_mapped_plane_pair.first = plane2;
    p2_detected_mapped_plane_pair.second = plane2;
  } else if (p2_p1_product > 0) {
    plane2_min_segment =
        PlaneUtils::check_point_neighbours((found_mapped_plane1->second).cloud_seg_map,
                                           plane2.cloud_seg_map) &&
        found_mapped_plane1->second.floor_level == plane2.floor_level;
    p2_detected_mapped_plane_pair.first = plane2;
    p2_detected_mapped_plane_pair.second = (found_mapped_plane1->second);
  } else {
    plane2_min_segment =
        PlaneUtils::check_point_neighbours((found_mapped_plane2->second).cloud_seg_map,
                                           plane2.cloud_seg_map) &&
        found_mapped_plane2->second.floor_level == plane2.floor_level;
    p2_detected_mapped_plane_pair.first = plane2;
    p2_detected_mapped_plane_pair.second = (found_mapped_plane2->second);
  }
  current_detected_mapped_plane_pairs.push_back(p2_detected_mapped_plane_pair);

  return current_detected_mapped_plane_pairs;
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
