#include <s_graphs/backend/room_mapper.hpp>

namespace s_graphs {

FiniteRoomMapper::FiniteRoomMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;
  room_information =
      node->get_parameter("room_information").get_parameter_value().get<double>();
  room_dist_threshold =
      node->get_parameter("room_dist_threshold").get_parameter_value().get<double>();
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

FiniteRoomMapper::~FiniteRoomMapper() {}

bool FiniteRoomMapper::lookup_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::RoomData room_data,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
    std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id) {
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

  float min_dist_room_x_inf_room = 100;
  s_graphs::InfiniteRooms matched_x_infinite_room;
  for (const auto& current_x_infinite_room : x_infinite_rooms) {
    if ((room_data.x_planes[0].id == current_x_infinite_room.second.plane1_id ||
         room_data.x_planes[0].id == current_x_infinite_room.second.plane2_id) &&
        (room_data.x_planes[1].id == current_x_infinite_room.second.plane1_id ||
         room_data.x_planes[1].id == current_x_infinite_room.second.plane2_id)) {
      min_dist_room_x_inf_room = 0;
      matched_x_infinite_room = current_x_infinite_room.second;
      break;
    }
    float dist_room_x_inf_room =
        sqrt(pow(room_data.room_center.position.x -
                     current_x_infinite_room.second.node->estimate().translation()(0),
                 2) +
             pow(room_data.room_center.position.y -
                     current_x_infinite_room.second.node->estimate().translation()(1),
                 2));
    if (dist_room_x_inf_room < min_dist_room_x_inf_room) {
      min_dist_room_x_inf_room = dist_room_x_inf_room;
      matched_x_infinite_room = current_x_infinite_room.second;
    }
  }

  float min_dist_room_y_inf_room = 100;
  s_graphs::InfiniteRooms matched_y_infinite_room;
  for (const auto& current_y_infinite_room : y_infinite_rooms) {
    if ((room_data.y_planes[0].id == current_y_infinite_room.second.plane1_id ||
         room_data.y_planes[0].id == current_y_infinite_room.second.plane2_id) &&
        (room_data.y_planes[1].id == current_y_infinite_room.second.plane1_id ||
         room_data.y_planes[1].id == current_y_infinite_room.second.plane2_id)) {
      min_dist_room_y_inf_room = 0;
      matched_y_infinite_room = current_y_infinite_room.second;
      break;
    }

    float dist_room_y_inf_room =
        sqrt(pow(room_data.room_center.position.x -
                     current_y_infinite_room.second.node->estimate().translation()(0),
                 2) +
             pow(room_data.room_center.position.y -
                     current_y_infinite_room.second.node->estimate().translation()(1),
                 2));
    if (dist_room_y_inf_room < min_dist_room_y_inf_room) {
      min_dist_room_y_inf_room = dist_room_y_inf_room;
      matched_y_infinite_room = current_y_infinite_room.second;
    }
  }

  auto found_x_plane1 = x_vert_planes.find(room_data.x_planes[0].id);
  auto found_x_plane2 = x_vert_planes.find(room_data.x_planes[1].id);
  auto found_y_plane1 = y_vert_planes.find(room_data.y_planes[0].id);
  auto found_y_plane2 = y_vert_planes.find(room_data.y_planes[1].id);

  if (min_dist_room_y_inf_room < 1.0 && min_dist_room_x_inf_room < 1.0) {
    std::cout << "Adding a room using mapped x and y infinite_room planes "
              << std::endl;
    map_room_from_existing_infinite_rooms(graph_slam,
                                          room_data,
                                          matched_x_infinite_room,
                                          matched_y_infinite_room,
                                          room_center,
                                          rooms_vec,
                                          x_vert_planes,
                                          y_vert_planes,
                                          (found_x_plane1->second),
                                          (found_x_plane2->second),
                                          (found_y_plane1->second),
                                          (found_y_plane1->second));
    remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE,
                                graph_slam,
                                matched_x_infinite_room,
                                x_infinite_rooms,
                                y_infinite_rooms);
    remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE,
                                graph_slam,
                                matched_y_infinite_room,
                                x_infinite_rooms,
                                y_infinite_rooms);
  } else if (min_dist_room_x_inf_room < 1.0 && min_dist_room_y_inf_room > 1.0) {
    map_room_from_existing_x_infinite_room(graph_slam,
                                           room_data,
                                           matched_x_infinite_room,
                                           room_center,
                                           rooms_vec,
                                           x_vert_planes,
                                           y_vert_planes,
                                           (found_x_plane1->second),
                                           (found_x_plane2->second),
                                           (found_y_plane1->second),
                                           (found_y_plane1->second));
    std::cout << "Will add room using mapped x infinite_room planes " << std::endl;
    remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE,
                                graph_slam,
                                matched_x_infinite_room,
                                x_infinite_rooms,
                                y_infinite_rooms);
  } else if (min_dist_room_y_inf_room < 1.0 && min_dist_room_x_inf_room > 1.0) {
    std::cout << "Will add room using mapped y infinite_room planes " << std::endl;
    map_room_from_existing_y_infinite_room(graph_slam,
                                           room_data,
                                           matched_y_infinite_room,
                                           room_center,
                                           rooms_vec,
                                           x_vert_planes,
                                           y_vert_planes,
                                           (found_x_plane1->second),
                                           (found_x_plane2->second),
                                           (found_y_plane1->second),
                                           (found_y_plane1->second));
    remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE,
                                graph_slam,
                                matched_y_infinite_room,
                                x_infinite_rooms,
                                y_infinite_rooms);
  }

  Eigen::Vector4d x_plane1(room_data.x_planes[0].nx,
                           room_data.x_planes[0].ny,
                           room_data.x_planes[0].nz,
                           room_data.x_planes[0].d);
  Eigen::Vector4d x_plane2(room_data.x_planes[1].nx,
                           room_data.x_planes[1].ny,
                           room_data.x_planes[1].nz,
                           room_data.x_planes[1].d);
  Eigen::Vector4d y_plane1(room_data.y_planes[0].nx,
                           room_data.y_planes[0].ny,
                           room_data.y_planes[0].nz,
                           room_data.y_planes[0].d);
  Eigen::Vector4d y_plane2(room_data.y_planes[1].nx,
                           room_data.y_planes[1].ny,
                           room_data.y_planes[1].nz,
                           room_data.y_planes[1].d);

  plane_data_list x_plane1_data, x_plane2_data;
  plane_data_list y_plane1_data, y_plane2_data;

  x_plane1_data.plane_id = room_data.x_planes[0].id;
  x_plane1_data.plane_unflipped = x_plane1;
  x_plane2_data.plane_id = room_data.x_planes[1].id;
  x_plane2_data.plane_unflipped = x_plane2;

  y_plane1_data.plane_id = room_data.y_planes[0].id;
  y_plane1_data.plane_unflipped = y_plane1;
  y_plane2_data.plane_id = room_data.y_planes[1].id;
  y_plane2_data.plane_unflipped = y_plane2;

  std::vector<plane_data_list> x_planes_room, y_planes_room;
  x_planes_room.push_back(x_plane1_data);
  x_planes_room.push_back(x_plane2_data);
  y_planes_room.push_back(y_plane1_data);
  y_planes_room.push_back(y_plane2_data);
  duplicate_found = factor_rooms(graph_slam,
                                 x_planes_room,
                                 y_planes_room,
                                 x_vert_planes,
                                 y_vert_planes,
                                 dupl_x_vert_planes,
                                 dupl_y_vert_planes,
                                 rooms_vec,
                                 room_id,
                                 room_center,
                                 room_data.cluster_array);

  return duplicate_found;
}

bool FiniteRoomMapper::factor_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    std::vector<plane_data_list> x_room_pair_vec,
    std::vector<plane_data_list> y_room_pair_vec,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
    std::unordered_map<int, Rooms>& rooms_vec,
    int& room_id,
    const Eigen::Isometry3d& room_center,
    const visualization_msgs::msg::MarkerArray& cluster_array) {
  g2o::VertexRoom* room_node;
  bool duplicate_found = false;
  int room_data_association;

  Eigen::Matrix<double, 1, 1> information_room_plane;
  information_room_plane(0, 0) = room_information;

  Eigen::Matrix<double, 2, 2> information_room_planes;
  information_room_planes.setZero();
  information_room_planes(0, 0) = room_information;
  information_room_planes(1, 1) = room_information;

  Eigen::Matrix<double, 3, 3> information_2planes;
  information_2planes.setZero();
  information_2planes(0, 0) = dupl_plane_matching_information;
  information_2planes(1, 1) = dupl_plane_matching_information;
  information_2planes(2, 2) = dupl_plane_matching_information;

  auto found_x_plane1 = x_vert_planes.begin();
  auto found_x_plane2 = x_vert_planes.begin();
  auto found_y_plane1 = y_vert_planes.begin();
  auto found_y_plane2 = y_vert_planes.begin();
  auto found_mapped_x_plane1 = x_vert_planes.begin();
  auto found_mapped_x_plane2 = x_vert_planes.begin();
  auto found_mapped_y_plane1 = y_vert_planes.begin();
  auto found_mapped_y_plane2 = y_vert_planes.begin();
  double x_plane1_meas, x_plane2_meas;
  double y_plane1_meas, y_plane2_meas;

  RCLCPP_DEBUG(node_obj->get_logger(),
               "room planes",
               "final room plane 1 %f %f %f %f",
               x_room_pair_vec[0].plane_unflipped.coeffs()(0),
               x_room_pair_vec[0].plane_unflipped.coeffs()(1),
               x_room_pair_vec[0].plane_unflipped.coeffs()(2),
               x_room_pair_vec[0].plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(),
               "room planes",
               "final room plane 2 %f %f %f %f",
               x_room_pair_vec[1].plane_unflipped.coeffs()(0),
               x_room_pair_vec[1].plane_unflipped.coeffs()(1),
               x_room_pair_vec[1].plane_unflipped.coeffs()(2),
               x_room_pair_vec[1].plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(),
               "room planes",
               "final room plane 3 %f %f %f %f",
               y_room_pair_vec[0].plane_unflipped.coeffs()(0),
               y_room_pair_vec[0].plane_unflipped.coeffs()(1),
               y_room_pair_vec[0].plane_unflipped.coeffs()(2),
               y_room_pair_vec[0].plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(),
               "room planes",
               "final room plane 4 %f %f %f %f",
               y_room_pair_vec[1].plane_unflipped.coeffs()(0),
               y_room_pair_vec[1].plane_unflipped.coeffs()(1),
               y_room_pair_vec[1].plane_unflipped.coeffs()(2),
               y_room_pair_vec[1].plane_unflipped.coeffs()(3));

  found_x_plane1 = x_vert_planes.find(x_room_pair_vec[0].plane_id);
  found_x_plane2 = x_vert_planes.find(x_room_pair_vec[1].plane_id);

  found_y_plane1 = y_vert_planes.find(y_room_pair_vec[0].plane_id);
  found_y_plane2 = y_vert_planes.find(y_room_pair_vec[1].plane_id);

  if (found_x_plane1 == x_vert_planes.end() || found_x_plane2 == x_vert_planes.end() ||
      found_y_plane1 == y_vert_planes.end() || found_y_plane2 == y_vert_planes.end()) {
    std::cout << "did not find a room plane in the plane vector" << std::endl;
    return duplicate_found;
  }

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  room_data_association = associate_rooms(room_center,
                                          rooms_vec,
                                          x_vert_planes,
                                          y_vert_planes,
                                          (found_x_plane1->second),
                                          (found_x_plane2->second),
                                          (found_y_plane1->second),
                                          (found_y_plane2->second),
                                          detected_mapped_plane_pairs);
  if ((rooms_vec.empty() || room_data_association == -1)) {
    std::cout << "found room with pose " << room_center.translation() << std::endl;
    room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_center);
    // room_node->setFixed(true);
    Rooms det_room;
    det_room.id = room_data_association;
    room_id = det_room.id;
    det_room.plane_x1 = x_room_pair_vec[0].plane_unflipped;
    det_room.plane_x2 = x_room_pair_vec[1].plane_unflipped;
    det_room.plane_y1 = y_room_pair_vec[0].plane_unflipped;
    det_room.plane_y2 = y_room_pair_vec[1].plane_unflipped;
    det_room.plane_x1_id = x_room_pair_vec[0].plane_id;
    det_room.plane_x2_id = x_room_pair_vec[1].plane_id;
    det_room.plane_y1_id = y_room_pair_vec[0].plane_id;
    det_room.plane_y2_id = y_room_pair_vec[1].plane_id;
    det_room.node = room_node;
    det_room.plane_x1_node = (found_x_plane1->second).plane_node;
    det_room.plane_x2_node = (found_x_plane2->second).plane_node;
    det_room.plane_y1_node = (found_y_plane1->second).plane_node;
    det_room.plane_y2_node = (found_y_plane2->second).plane_node;
    det_room.cluster_array = cluster_array;
    det_room.local_graph = std::make_shared<GraphSLAM>();
    rooms_vec.insert({det_room.id, det_room});

    auto edge_room_planes =
        graph_slam->add_room_4planes_edge(room_node,
                                          (found_x_plane1->second).plane_node,
                                          (found_x_plane2->second).plane_node,
                                          (found_y_plane1->second).plane_node,
                                          (found_y_plane2->second).plane_node,
                                          information_room_planes);
    graph_slam->add_robust_kernel(edge_room_planes, "Huber", 1.0);
  } else {
    /* add the edge between detected planes and the infinite_room */
    room_node = rooms_vec[room_data_association].node;
    room_id = room_data_association;
    std::cout << "Matched det room with pose " << room_center.translation()
              << " to mapped room with id " << room_data_association << " and pose "
              << room_node->estimate().translation() << std::endl;

    /*update the cluster array */
    rooms_vec[room_data_association].cluster_array = cluster_array;

    std::set<g2o::HyperGraph::Edge*> xplane1_edges =
        (found_x_plane1->second).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> xplane2_edges =
        (found_x_plane2->second).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> yplane1_edges =
        (found_y_plane1->second).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> yplane2_edges =
        (found_y_plane2->second).plane_node->edges();

    if (detected_mapped_plane_pairs[0].first.id !=
        detected_mapped_plane_pairs[0].second.id) {
      if (!MapperUtils::check_plane_ids(
              xplane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
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
              xplane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(
            detected_mapped_plane_pairs[1].first.plane_node,
            detected_mapped_plane_pairs[1].second.plane_node,
            information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        duplicate_found = true;
        std::cout << "Adding new x2 plane " << std::endl;
      }
    }
    if (detected_mapped_plane_pairs[2].first.id !=
        detected_mapped_plane_pairs[2].second.id) {
      if (!MapperUtils::check_plane_ids(
              yplane1_edges, detected_mapped_plane_pairs[2].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(
            detected_mapped_plane_pairs[2].first.plane_node,
            detected_mapped_plane_pairs[2].second.plane_node,
            information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        duplicate_found = true;
        std::cout << "Adding new y1 plane " << std::endl;
      }
    }
    if (detected_mapped_plane_pairs[3].first.id !=
        detected_mapped_plane_pairs[3].second.id) {
      if (!MapperUtils::check_plane_ids(
              yplane2_edges, detected_mapped_plane_pairs[3].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(
            detected_mapped_plane_pairs[3].first.plane_node,
            detected_mapped_plane_pairs[3].second.plane_node,
            information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        duplicate_found = true;
        std::cout << "Adding new y2 plane " << std::endl;
      }
    }
  }

  return duplicate_found;
  // std::cout << "found xplane1 id : " << (*found_x_plane1).id << std::endl;
  // std::cout << "found xplane1 coeffs : " <<
  // (*found_x_plane1).plane_node->estimate().coeffs() << std::endl; std::cout << "found
  // xplane2 id : " << (*found_x_plane2).id << std::endl; std::cout << "found xplane2
  // coeffs : " << (*found_x_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "found yplane1 id : " << (*found_y_plane1).id << std::endl;
  // std::cout << "found yplane1 coeffs : " <<
  // (*found_y_plane1).plane_node->estimate().coeffs() << std::endl; std::cout << "found
  // yplane2 id : " << (*found_y_plane2).id << std::endl; std::cout << "found yplane2
  // coeffs : " << (*found_y_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "mapped xplane1 id : " << (*found_mapped_x_plane1).id << std::endl;
  // std::cout << "mapped xplane1 coeffs : " <<
  // (*found_mapped_x_plane1).plane_node->estimate().coeffs() << std::endl; std::cout <<
  // "mapped xplane2 id : " << (*found_mapped_x_plane2).id << std::endl; std::cout <<
  // "mapped xplane2 coeffs : " <<
  // (*found_mapped_x_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "mapped yplane1 id : " << (*found_mapped_y_plane1).id << std::endl;
  // std::cout << "mapped yplane1 coeffs : " <<
  // (*found_mapped_y_plane1).plane_node->estimate().coeffs() << std::endl; std::cout <<
  // "mapped yplane2 id : " << (*found_mapped_y_plane2).id << std::endl; std::cout <<
  // "mapped yplane2 coeffs : " <<
  // (*found_mapped_y_plane2).plane_node->estimate().coeffs() << std::endl;
}

int FiniteRoomMapper::associate_rooms(
    const Eigen::Isometry3d& room_center,
    const std::unordered_map<int, Rooms>& rooms_vec,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const VerticalPlanes& x_plane1,
    const VerticalPlanes& x_plane2,
    const VerticalPlanes& y_plane1,
    const VerticalPlanes& y_plane2,
    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
        detected_mapped_plane_pairs) {
  float min_dist = 100;
  int data_association;
  data_association = -1;
  bool x_plane1_min_segment = false, x_plane2_min_segment = false;
  bool y_plane1_min_segment = false, y_plane2_min_segment = false;

  for (const auto& room : rooms_vec) {
    float diff_x =
        room_center.translation()(0) - room.second.node->estimate().translation()(0);
    float diff_y =
        room_center.translation()(1) - room.second.node->estimate().translation()(1);
    float dist = sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
    RCLCPP_DEBUG(node_obj->get_logger(), "room planes", "dist room %f", dist);

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
        current_detected_mapped_plane_pairs;
    auto found_mapped_xplane1 = x_vert_planes.find(room.second.plane_x1_id);
    auto found_mapped_xplane2 = x_vert_planes.find(room.second.plane_x2_id);

    std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
    std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;

    if (x_plane1.id == (found_mapped_xplane1->second).id ||
        x_plane1.id == (found_mapped_xplane2->second).id) {
      x_plane1_min_segment = true;
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = x_plane1;
    } else if ((x_plane1).plane_node->estimate().coeffs().head(3).dot(
                   (found_mapped_xplane1->second)
                       .plane_node->estimate()
                       .coeffs()
                       .head(3)) > 0) {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_xplane1->second).plane, x_plane1.plane);
      if (maha_dist < 0.5) x_plane1_min_segment = true;
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = (found_mapped_xplane1->second);

    } else {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_xplane2->second).plane, x_plane1.plane);
      if (maha_dist < 0.5) x_plane1_min_segment = true;
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = (found_mapped_xplane2->second);
    }

    current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);
    if (x_plane2.id == (found_mapped_xplane1->second).id ||
        x_plane2.id == (found_mapped_xplane2->second).id) {
      x_plane2_min_segment = true;
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = x_plane2;
    } else if ((x_plane2).plane_node->estimate().coeffs().head(3).dot(
                   (found_mapped_xplane1->second)
                       .plane_node->estimate()
                       .coeffs()
                       .head(3)) > 0) {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_xplane1->second).plane, x_plane2.plane);
      if (maha_dist < 0.5) x_plane2_min_segment = true;
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = (found_mapped_xplane1->second);

    } else {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_xplane2->second).plane, x_plane2.plane);
      if (maha_dist < 0.5) x_plane2_min_segment = true;
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = (found_mapped_xplane2->second);
    }
    current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

    auto found_mapped_yplane1 = y_vert_planes.find(room.second.plane_y1_id);
    auto found_mapped_yplane2 = y_vert_planes.find(room.second.plane_y2_id);

    std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
    std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;

    if (y_plane1.id == (found_mapped_yplane1->second).id ||
        y_plane1.id == (found_mapped_yplane2->second).id) {
      y_plane1_min_segment = true;
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = y_plane1;
    } else if ((y_plane1).plane_node->estimate().coeffs().head(3).dot(
                   (found_mapped_yplane1->second)
                       .plane_node->estimate()
                       .coeffs()
                       .head(3)) > 0) {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_yplane1->second).plane, y_plane1.plane);
      if (maha_dist < 0.5) y_plane1_min_segment = true;
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = (found_mapped_yplane1->second);

    } else {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_yplane2->second).plane, y_plane1.plane);
      if (maha_dist < 0.5) y_plane1_min_segment = true;
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = (found_mapped_yplane2->second);
    }
    current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

    if (y_plane2.id == (found_mapped_yplane1->second).id ||
        y_plane2.id == (found_mapped_yplane2->second).id) {
      y_plane2_min_segment = true;
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = y_plane2;
    } else if ((y_plane2).plane_node->estimate().coeffs().head(3).dot(
                   (found_mapped_yplane1->second)
                       .plane_node->estimate()
                       .coeffs()
                       .head(3)) > 0) {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_yplane1->second).plane, y_plane2.plane);
      if (maha_dist < 0.5) y_plane2_min_segment = true;
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = (found_mapped_yplane1->second);

    } else {
      double maha_dist = PlaneUtils::plane_difference(
          (found_mapped_yplane2->second).plane, y_plane2.plane);
      if (maha_dist < 0.5) y_plane2_min_segment = true;
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = (found_mapped_yplane2->second);
    }
    current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

    if (dist < min_dist && (x_plane1_min_segment && x_plane2_min_segment &&
                            y_plane1_min_segment && y_plane2_min_segment)) {
      min_dist = dist;
      data_association = room.second.id;
      detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
    }
  }

  RCLCPP_DEBUG(node_obj->get_logger(), "room planes", "min dist room %f", min_dist);
  if (min_dist > room_dist_threshold) data_association = -1;

  return data_association;
}

bool FiniteRoomMapper::check_room_ids(
    const int plane_type,
    const std::set<g2o::HyperGraph::Edge*>& plane_edges,
    const g2o::VertexRoom* room_node) {
  for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom4Planes* edge_room_4planes =
          dynamic_cast<g2o::EdgeRoom4Planes*>(*edge_itr);
      if (edge_room_4planes) {
        g2o::VertexRoom* found_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_room_4planes->vertices()[0]);
        if (found_room_node->id() == room_node->id()) return true;
      }
    }

    if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom4Planes* edge_room_4planes =
          dynamic_cast<g2o::EdgeRoom4Planes*>(*edge_itr);
      if (edge_room_4planes) {
        g2o::VertexRoom* found_room_node =
            dynamic_cast<g2o::VertexRoom*>(edge_room_4planes->vertices()[0]);
        if (found_room_node->id() == room_node->id()) return true;
      }
    }
  }

  return false;
}

void FiniteRoomMapper::map_room_from_existing_infinite_rooms(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::RoomData& det_room_data,
    const s_graphs::InfiniteRooms& matched_x_infinite_room,
    const s_graphs::InfiniteRooms& matched_y_infinite_room,
    const Eigen::Isometry3d& room_center,
    std::unordered_map<int, Rooms>& rooms_vec,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const VerticalPlanes& x_plane1,
    const VerticalPlanes& x_plane2,
    const VerticalPlanes& y_plane1,
    const VerticalPlanes& y_plane2) {
  g2o::VertexRoom* room_node;
  int room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  room_data_association = associate_rooms(room_center,
                                          rooms_vec,
                                          x_vert_planes,
                                          y_vert_planes,
                                          x_plane1,
                                          x_plane2,
                                          y_plane1,
                                          y_plane2,
                                          detected_mapped_plane_pairs);
  if ((rooms_vec.empty() || room_data_association == -1)) {
    std::cout << "Add a room using mapped x and y infinite_rooms at pose"
              << room_center.translation() << std::endl;
    room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_center);
    Rooms det_room;
    det_room.id = room_data_association;
    det_room.plane_x1 = matched_x_infinite_room.plane1;
    det_room.plane_x2 = matched_x_infinite_room.plane2;
    det_room.plane_y1 = matched_y_infinite_room.plane1;
    det_room.plane_y2 = matched_y_infinite_room.plane2;
    det_room.plane_x1_id = matched_x_infinite_room.plane1_id;
    det_room.plane_x2_id = matched_x_infinite_room.plane2_id;
    det_room.plane_y1_id = matched_y_infinite_room.plane1_id;
    det_room.plane_y2_id = matched_y_infinite_room.plane2_id;
    det_room.plane_x1_node = matched_x_infinite_room.plane1_node;
    det_room.plane_x2_node = matched_x_infinite_room.plane2_node;
    det_room.plane_y1_node = matched_y_infinite_room.plane1_node;
    det_room.plane_y2_node = matched_y_infinite_room.plane2_node;
    det_room.local_graph = std::make_shared<GraphSLAM>();
    det_room.node = room_node;
    for (int i = 0; i < matched_x_infinite_room.cluster_array.markers.size(); ++i)
      det_room.cluster_array.markers.push_back(
          matched_x_infinite_room.cluster_array.markers[i]);
    for (int i = 0; i < matched_y_infinite_room.cluster_array.markers.size(); ++i)
      det_room.cluster_array.markers.push_back(
          matched_y_infinite_room.cluster_array.markers[i]);
    rooms_vec.insert({det_room.id, det_room});
    return;
  } else
    return;
}

void FiniteRoomMapper::map_room_from_existing_x_infinite_room(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::RoomData& det_room_data,
    const s_graphs::InfiniteRooms& matched_x_infinite_room,
    const Eigen::Isometry3d& room_center,
    std::unordered_map<int, Rooms>& rooms_vec,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const VerticalPlanes& x_plane1,
    const VerticalPlanes& x_plane2,
    const VerticalPlanes& y_plane1,
    const VerticalPlanes& y_plane2) {
  g2o::VertexRoom* room_node;
  int room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  room_data_association = associate_rooms(room_center,
                                          rooms_vec,
                                          x_vert_planes,
                                          y_vert_planes,
                                          x_plane1,
                                          x_plane2,
                                          y_plane1,
                                          y_plane2,
                                          detected_mapped_plane_pairs);
  if ((rooms_vec.empty() || room_data_association == -1)) {
    std::cout << "Add a room using mapped x infinite_rooms planes at pose"
              << room_center.translation() << std::endl;
    room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_center);
    Rooms det_room;
    det_room.id = room_data_association;
    det_room.plane_x1 = matched_x_infinite_room.plane1;
    det_room.plane_x2 = matched_x_infinite_room.plane2;
    Eigen::Vector4d y_plane1(det_room_data.y_planes[0].nx,
                             det_room_data.y_planes[0].ny,
                             det_room_data.y_planes[0].nz,
                             det_room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(det_room_data.y_planes[1].nx,
                             det_room_data.y_planes[1].ny,
                             det_room_data.y_planes[1].nz,
                             det_room_data.y_planes[1].d);
    det_room.plane_y1 = y_plane1;
    det_room.plane_y2 = y_plane2;
    det_room.plane_x1_id = matched_x_infinite_room.plane1_id;
    det_room.plane_x2_id = matched_x_infinite_room.plane2_id;
    det_room.plane_y1_id = det_room_data.y_planes[0].id;
    det_room.plane_y2_id = det_room_data.y_planes[1].id;
    det_room.plane_x1_node = matched_x_infinite_room.plane1_node;
    det_room.plane_x2_node = matched_x_infinite_room.plane2_node;

    auto found_y_plane1 = y_vert_planes.find(det_room_data.y_planes[0].id);
    auto found_y_plane2 = y_vert_planes.find(det_room_data.y_planes[1].id);
    det_room.plane_y1_node = (found_y_plane1->second).plane_node;
    det_room.plane_y2_node = (found_y_plane2->second).plane_node;

    for (int i = 0; i < matched_x_infinite_room.cluster_array.markers.size(); ++i)
      det_room.cluster_array.markers.push_back(
          matched_x_infinite_room.cluster_array.markers[i]);
    det_room.local_graph = std::make_shared<GraphSLAM>();
    det_room.node = room_node;
    rooms_vec.insert({det_room.id, det_room});
    return;
  } else
    return;
}

void FiniteRoomMapper::map_room_from_existing_y_infinite_room(
    std::shared_ptr<GraphSLAM>& graph_slam,
    const situational_graphs_msgs::msg::RoomData& det_room_data,
    const s_graphs::InfiniteRooms& matched_y_infinite_room,
    const Eigen::Isometry3d& room_center,
    std::unordered_map<int, Rooms>& rooms_vec,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const VerticalPlanes& x_plane1,
    const VerticalPlanes& x_plane2,
    const VerticalPlanes& y_plane1,
    const VerticalPlanes& y_plane2) {
  g2o::VertexRoom* room_node;
  int room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  room_data_association = associate_rooms(room_center,
                                          rooms_vec,
                                          x_vert_planes,
                                          y_vert_planes,
                                          x_plane1,
                                          x_plane2,
                                          y_plane1,
                                          y_plane2,
                                          detected_mapped_plane_pairs);
  if ((rooms_vec.empty() || room_data_association == -1)) {
    std::cout << "Add a room using mapped y infinite_rooms planes at pose"
              << room_center.translation() << std::endl;
    room_data_association = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_center);
    Rooms det_room;
    det_room.id = room_data_association;
    Eigen::Vector4d x_plane1(det_room_data.x_planes[0].nx,
                             det_room_data.x_planes[0].ny,
                             det_room_data.x_planes[0].nz,
                             det_room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(det_room_data.x_planes[1].nx,
                             det_room_data.x_planes[1].ny,
                             det_room_data.x_planes[1].nz,
                             det_room_data.x_planes[1].d);
    det_room.plane_x1 = x_plane1;
    det_room.plane_x2 = x_plane2;
    det_room.plane_y1 = matched_y_infinite_room.plane1;
    det_room.plane_y2 = matched_y_infinite_room.plane2;
    det_room.plane_x1_id = det_room_data.x_planes[0].id;
    det_room.plane_x2_id = det_room_data.x_planes[1].id;
    det_room.plane_y1_id = matched_y_infinite_room.plane1_id;
    det_room.plane_y2_id = matched_y_infinite_room.plane2_id;
    det_room.plane_x1_id = det_room_data.x_planes[0].id;
    det_room.plane_x2_id = det_room_data.x_planes[1].id;
    auto found_x_plane1 = x_vert_planes.find(det_room_data.x_planes[0].id);
    auto found_x_plane2 = x_vert_planes.find(det_room_data.x_planes[1].id);

    det_room.plane_x1_node = (found_x_plane1->second).plane_node;
    det_room.plane_x2_node = (found_x_plane2->second).plane_node;
    det_room.plane_y1_node = matched_y_infinite_room.plane1_node;
    det_room.plane_y2_node = matched_y_infinite_room.plane2_node;

    for (int i = 0; i < matched_y_infinite_room.cluster_array.markers.size(); ++i)
      det_room.cluster_array.markers.push_back(
          matched_y_infinite_room.cluster_array.markers[i]);
    det_room.local_graph = std::make_shared<GraphSLAM>();
    det_room.node = room_node;
    rooms_vec.insert({det_room.id, det_room});
    return;
  } else
    return;
}

void FiniteRoomMapper::remove_mapped_infinite_room(
    const int plane_type,
    std::shared_ptr<GraphSLAM>& graph_slam,
    s_graphs::InfiniteRooms matched_infinite_room,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms) {
  std::set<g2o::HyperGraph::Edge*> edges = matched_infinite_room.node->edges();
  for (auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
    g2o::EdgeRoom2Planes* edge_room_2planes =
        dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
    if (edge_room_2planes) {
      if (graph_slam->remove_room_2planes_edge(edge_room_2planes))
        std::cout << "removed edge - room-2planes " << std::endl;
      continue;
    }
  }

  if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if (graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = x_infinite_rooms.find(matched_infinite_room.id);
      x_infinite_rooms.erase(mapped_infinite_room);
      std::cout << "removed overlapped x-infinite_room " << std::endl;
    }
  } else if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if (graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = y_infinite_rooms.find(matched_infinite_room.id);
      y_infinite_rooms.erase(mapped_infinite_room);
      std::cout << "removed overlapped y-infinite_room " << std::endl;
    }
  }
}

}  // namespace s_graphs
