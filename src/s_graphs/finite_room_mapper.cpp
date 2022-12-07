// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_mapper.hpp>

namespace s_graphs {

FiniteRoomMapper::FiniteRoomMapper(const ros::NodeHandle& private_nh) {
  plane_utils.reset(new PlaneUtils());

  room_information = private_nh.param<double>("room_information", 0.01);
  room_dist_threshold = private_nh.param<double>("room_dist_threshold", 1.0);
  room_point_diff_threshold = private_nh.param<double>("room_point_diff_threshold", 3.0);
  room_width_diff_threshold = private_nh.param<double>("room_width_diff_threshold", 2.5);
  room_plane_length_diff_threshold = private_nh.param<double>("room_plane_length_diff_threshold", 0.3);
  dupl_plane_matching_information = private_nh.param<double>("dupl_plane_matching_information", 0.01);

  room_min_width = private_nh.param<double>("room_min_width", 2.5);
  room_max_width = private_nh.param<double>("room_max_width", 6.0);

  use_parallel_plane_constraint = private_nh.param<bool>("use_parallel_plane_constraint", true);
  use_perpendicular_plane_constraint = private_nh.param<bool>("use_perpendicular_plane_constraint", true);
}

FiniteRoomMapper::~FiniteRoomMapper() {}

void FiniteRoomMapper::lookup_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const std::vector<plane_data_list>& x_det_room_candidates, const std::vector<plane_data_list>& y_det_room_candidates, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Rooms>& rooms_vec) {
  std::vector<structure_data_list> x_room_pair_vec = sort_rooms(PlaneUtils::plane_class::X_VERT_PLANE, x_det_room_candidates);
  std::vector<structure_data_list> y_room_pair_vec = sort_rooms(PlaneUtils::plane_class::Y_VERT_PLANE, y_det_room_candidates);
  std::pair<std::vector<plane_data_list>, std::vector<plane_data_list>> refined_room_pair = refine_rooms(x_room_pair_vec, y_room_pair_vec);

  if(refined_room_pair.first.size() == 2 && refined_room_pair.second.size() == 2) {
    factor_rooms(graph_slam, refined_room_pair.first, refined_room_pair.second, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, rooms_vec);
  }
}

void FiniteRoomMapper::lookup_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<InfiniteRooms>& x_infinite_rooms, std::vector<InfiniteRooms>& y_infinite_rooms, std::vector<Rooms>& rooms_vec) {
  float min_dist_room_x_corr = 100;
  s_graphs::InfiniteRooms matched_x_infinite_room;
  for(const auto& current_x_infinite_room : x_infinite_rooms) {
    if((room_data.x_planes[0].id == current_x_infinite_room.plane1_id || room_data.x_planes[0].id == current_x_infinite_room.plane2_id) && (room_data.x_planes[1].id == current_x_infinite_room.plane1_id || room_data.x_planes[1].id == current_x_infinite_room.plane2_id)) {
      min_dist_room_x_corr = 0;
      matched_x_infinite_room = current_x_infinite_room;
      break;
    }
    float dist_room_x_corr = sqrt(pow(room_data.room_center.x - current_x_infinite_room.node->estimate()(0), 2) + pow(room_data.room_center.y - current_x_infinite_room.node->estimate()(1), 2));
    if(dist_room_x_corr < min_dist_room_x_corr) {
      min_dist_room_x_corr = dist_room_x_corr;
      matched_x_infinite_room = current_x_infinite_room;
    }
  }

  float min_dist_room_y_corr = 100;
  s_graphs::InfiniteRooms matched_y_infinite_room;
  for(const auto& current_y_infinite_room : y_infinite_rooms) {
    if((room_data.y_planes[0].id == current_y_infinite_room.plane1_id || room_data.y_planes[0].id == current_y_infinite_room.plane2_id) && (room_data.y_planes[1].id == current_y_infinite_room.plane1_id || room_data.y_planes[1].id == current_y_infinite_room.plane2_id)) {
      min_dist_room_y_corr = 0;
      matched_y_infinite_room = current_y_infinite_room;
      break;
    }

    float dist_room_y_corr = sqrt(pow(room_data.room_center.x - current_y_infinite_room.node->estimate()(0), 2) + pow(room_data.room_center.y - current_y_infinite_room.node->estimate()(1), 2));
    if(dist_room_y_corr < min_dist_room_y_corr) {
      min_dist_room_y_corr = dist_room_y_corr;
      matched_y_infinite_room = current_y_infinite_room;
    }
  }

  auto found_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[0].id);
  auto found_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[1].id);
  auto found_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[0].id);
  auto found_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[1].id);

  if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr < 1.0) {
    std::cout << "Adding a room using mapped x and y infinite_room planes " << std::endl;
    map_room_from_existing_infinite_rooms(graph_slam, room_data, matched_x_infinite_room, matched_y_infinite_room, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE, graph_slam, matched_x_infinite_room, x_infinite_rooms, y_infinite_rooms);
    remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE, graph_slam, matched_y_infinite_room, x_infinite_rooms, y_infinite_rooms);
  } else if(min_dist_room_x_corr < 1.0 && min_dist_room_y_corr > 1.0) {
    map_room_from_existing_x_infinite_room(graph_slam, room_data, matched_x_infinite_room, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    std::cout << "Will add room using mapped x infinite_room planes " << std::endl;
    remove_mapped_infinite_room(PlaneUtils::plane_class::X_VERT_PLANE, graph_slam, matched_x_infinite_room, x_infinite_rooms, y_infinite_rooms);
  } else if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr > 1.0) {
    std::cout << "Will add room using mapped y infinite_room planes " << std::endl;
    map_room_from_existing_y_infinite_room(graph_slam, room_data, matched_y_infinite_room, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    remove_mapped_infinite_room(PlaneUtils::plane_class::Y_VERT_PLANE, graph_slam, matched_y_infinite_room, x_infinite_rooms, y_infinite_rooms);
  }

  Eigen::Vector4d x_plane1(room_data.x_planes[0].nx, room_data.x_planes[0].ny, room_data.x_planes[0].nz, room_data.x_planes[0].d);
  Eigen::Vector4d x_plane2(room_data.x_planes[1].nx, room_data.x_planes[1].ny, room_data.x_planes[1].nz, room_data.x_planes[1].d);
  Eigen::Vector4d y_plane1(room_data.y_planes[0].nx, room_data.y_planes[0].ny, room_data.y_planes[0].nz, room_data.y_planes[0].d);
  Eigen::Vector4d y_plane2(room_data.y_planes[1].nx, room_data.y_planes[1].ny, room_data.y_planes[1].nz, room_data.y_planes[1].d);

  plane_data_list x_plane1_data, x_plane2_data;
  plane_data_list y_plane1_data, y_plane2_data;

  x_plane1_data.plane_id = room_data.x_planes[0].id;
  x_plane1_data.plane_unflipped = x_plane1;
  x_plane1_data.plane_centroid(0) = room_data.room_center.x;
  x_plane1_data.plane_centroid(1) = room_data.room_center.y;
  x_plane2_data.plane_id = room_data.x_planes[1].id;
  x_plane2_data.plane_unflipped = x_plane2;

  y_plane1_data.plane_id = room_data.y_planes[0].id;
  y_plane1_data.plane_unflipped = y_plane1;
  y_plane2_data.plane_id = room_data.y_planes[1].id;
  y_plane2_data.plane_unflipped = y_plane2;

  // get the room neighbours
  x_plane1_data.connected_id = room_data.id;
  for(const auto& room_neighbour_id : room_data.neighbour_ids) {
    x_plane1_data.connected_neighbour_ids.push_back(room_neighbour_id);
  }

  std::vector<plane_data_list> x_planes_room, y_planes_room;
  x_planes_room.push_back(x_plane1_data);
  x_planes_room.push_back(x_plane2_data);
  y_planes_room.push_back(y_plane1_data);
  y_planes_room.push_back(y_plane2_data);
  factor_rooms(graph_slam, x_planes_room, y_planes_room, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, rooms_vec);
}

std::vector<structure_data_list> FiniteRoomMapper::sort_rooms(const int& plane_type, const std::vector<plane_data_list>& room_candidates) {
  std::vector<structure_data_list> room_pair_vec;

  for(int i = 0; i < room_candidates.size(); ++i) {
    for(int j = i + 1; j < room_candidates.size(); ++j) {
      float room_width = plane_utils->width_between_planes(room_candidates[i].plane_unflipped.coeffs(), room_candidates[j].plane_unflipped.coeffs());
      float diff_plane_length = fabs(room_candidates[i].plane_length - room_candidates[j].plane_length);
      float start_point_diff = MapperUtils::point_difference(plane_type, room_candidates[i].start_point, room_candidates[j].start_point);
      float end_point_diff = MapperUtils::point_difference(plane_type, room_candidates[i].end_point, room_candidates[j].end_point);
      float avg_plane_point_diff = (start_point_diff + end_point_diff) / 2;
      ROS_DEBUG_NAMED("room planes", "room plane i coeffs %f %f %f %f", room_candidates[i].plane_unflipped.coeffs()(0), room_candidates[i].plane_unflipped.coeffs()(1), room_candidates[i].plane_unflipped.coeffs()(2), room_candidates[i].plane_unflipped.coeffs()(3));
      ROS_DEBUG_NAMED("room planes", "room plane j coeffs %f %f %f %f", room_candidates[j].plane_unflipped.coeffs()(0), room_candidates[j].plane_unflipped.coeffs()(1), room_candidates[j].plane_unflipped.coeffs()(2), room_candidates[j].plane_unflipped.coeffs()(3));
      ROS_DEBUG_NAMED("room planes", "room width %f", room_width);
      ROS_DEBUG_NAMED("room planes", "room plane lenght diff %f", diff_plane_length);
      ROS_DEBUG_NAMED("room planes", "room plane point diff %f", avg_plane_point_diff);

      if(room_candidates[i].plane_unflipped.coeffs().head(3).dot(room_candidates[j].plane_unflipped.coeffs().head(3)) < 0 && (room_width > room_min_width && room_width < room_max_width) && diff_plane_length < room_plane_length_diff_threshold) {
        if(avg_plane_point_diff < room_point_diff_threshold) {
          structure_data_list room_pair;
          room_pair.plane1 = room_candidates[i];
          room_pair.plane2 = room_candidates[j];
          room_pair.width = room_width;
          room_pair.length_diff = diff_plane_length;
          room_pair.avg_point_diff = avg_plane_point_diff;
          room_pair_vec.push_back(room_pair);
          ROS_DEBUG_NAMED("room planes", "adding room candidates");
        }
      }
    }
  }
  return room_pair_vec;
}

std::pair<std::vector<plane_data_list>, std::vector<plane_data_list>> FiniteRoomMapper::refine_rooms(std::vector<structure_data_list> x_room_vec, std::vector<structure_data_list> y_room_vec) {
  float min_room_point_diff = room_point_diff_threshold;
  std::vector<plane_data_list> x_room, y_room;
  x_room.resize(2);
  y_room.resize(2);

  for(int i = 0; i < x_room_vec.size(); ++i) {
    for(int j = 0; j < y_room_vec.size(); ++j) {
      float width_diff = fabs(x_room_vec[i].width - y_room_vec[j].width);
      if(width_diff < room_width_diff_threshold) {
        float room_diff = (x_room_vec[i].avg_point_diff + y_room_vec[j].avg_point_diff) / 2;
        if(room_diff < min_room_point_diff) {
          min_room_point_diff = room_diff;
          x_room[0] = x_room_vec[i].plane1;
          x_room[1] = x_room_vec[i].plane2;
          y_room[0] = y_room_vec[j].plane1;
          y_room[1] = y_room_vec[j].plane2;
        }
      }
    }
  }

  if(min_room_point_diff >= room_point_diff_threshold) {
    std::vector<plane_data_list> x_room_empty, y_room_empty;
    x_room_empty.resize(0);
    y_room_empty.resize(0);
    return std::make_pair(x_room_empty, x_room_empty);
  } else
    return std::make_pair(x_room, y_room);
}

void FiniteRoomMapper::factor_rooms(std::unique_ptr<GraphSLAM>& graph_slam, std::vector<plane_data_list> x_room_pair_vec, std::vector<plane_data_list> y_room_pair_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Rooms>& rooms_vec) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

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

  ROS_DEBUG_NAMED("room planes", "final room plane 1 %f %f %f %f", x_room_pair_vec[0].plane_unflipped.coeffs()(0), x_room_pair_vec[0].plane_unflipped.coeffs()(1), x_room_pair_vec[0].plane_unflipped.coeffs()(2), x_room_pair_vec[0].plane_unflipped.coeffs()(3));
  ROS_DEBUG_NAMED("room planes", "final room plane 2 %f %f %f %f", x_room_pair_vec[1].plane_unflipped.coeffs()(0), x_room_pair_vec[1].plane_unflipped.coeffs()(1), x_room_pair_vec[1].plane_unflipped.coeffs()(2), x_room_pair_vec[1].plane_unflipped.coeffs()(3));
  ROS_DEBUG_NAMED("room planes", "final room plane 3 %f %f %f %f", y_room_pair_vec[0].plane_unflipped.coeffs()(0), y_room_pair_vec[0].plane_unflipped.coeffs()(1), y_room_pair_vec[0].plane_unflipped.coeffs()(2), y_room_pair_vec[0].plane_unflipped.coeffs()(3));
  ROS_DEBUG_NAMED("room planes", "final room plane 4 %f %f %f %f", y_room_pair_vec[1].plane_unflipped.coeffs()(0), y_room_pair_vec[1].plane_unflipped.coeffs()(1), y_room_pair_vec[1].plane_unflipped.coeffs()(2), y_room_pair_vec[1].plane_unflipped.coeffs()(3));

  found_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_room_pair_vec[0].plane_id);
  found_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_room_pair_vec[1].plane_id);
  found_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_room_pair_vec[0].plane_id);
  found_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_room_pair_vec[1].plane_id);

  if(found_x_plane1 == x_vert_planes.end() || found_x_plane2 == x_vert_planes.end() || found_y_plane1 == y_vert_planes.end() || found_y_plane2 == y_vert_planes.end()) {
    std::cout << "did not find a room plane in the plane vector" << std::endl;
    return;
  }

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  Eigen::Vector2d room_pose(x_room_pair_vec[0].plane_centroid(0), x_room_pair_vec[0].plane_centroid(1));
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane2), detected_mapped_plane_pairs);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "found room with pose " << room_pose << std::endl;
    room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_pose);
    // room_node->setFixed(true);
    Rooms det_room;
    det_room.id = room_data_association.first;
    det_room.plane_x1 = x_room_pair_vec[0].plane_unflipped;
    det_room.plane_x2 = x_room_pair_vec[1].plane_unflipped;
    det_room.plane_y1 = y_room_pair_vec[0].plane_unflipped;
    det_room.plane_y2 = y_room_pair_vec[1].plane_unflipped;
    det_room.plane_x1_id = x_room_pair_vec[0].plane_id;
    det_room.plane_x2_id = x_room_pair_vec[1].plane_id;
    det_room.plane_y1_id = y_room_pair_vec[0].plane_id;
    det_room.plane_y2_id = y_room_pair_vec[1].plane_id;
    det_room.connected_id = x_room_pair_vec[0].connected_id;
    det_room.connected_neighbour_ids = x_room_pair_vec[0].connected_neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);

    auto edge_room_planes = graph_slam->add_room_4planes_edge(room_node, (*found_x_plane1).plane_node, (*found_x_plane2).plane_node, (*found_y_plane1).plane_node, (*found_y_plane2).plane_node, information_room_planes);
    graph_slam->add_robust_kernel(edge_room_planes, "Huber", 1.0);
  } else {
    /* add the edge between detected planes and the infinite_room */
    room_node = rooms_vec[room_data_association.second].node;
    std::cout << "Matched det room with pose " << room_pose << " to mapped room with id " << room_data_association.first << " and pose " << room_node->estimate() << std::endl;

    std::set<g2o::HyperGraph::Edge*> xplane1_edges = (*found_x_plane1).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> xplane2_edges = (*found_x_plane2).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> yplane1_edges = (*found_y_plane1).plane_node->edges();
    std::set<g2o::HyperGraph::Edge*> yplane2_edges = (*found_y_plane2).plane_node->edges();

    if(detected_mapped_plane_pairs[0].first.id != detected_mapped_plane_pairs[0].second.id) {
      if(!MapperUtils::check_plane_ids(xplane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[0].first.plane_node, detected_mapped_plane_pairs[0].second.plane_node, information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        std::cout << "Adding new x1 plane " << std::endl;
      }
    }
    if(detected_mapped_plane_pairs[1].first.id != detected_mapped_plane_pairs[1].second.id) {
      if(!MapperUtils::check_plane_ids(xplane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[1].first.plane_node, detected_mapped_plane_pairs[1].second.plane_node, information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        std::cout << "Adding new x2 plane " << std::endl;
      }
    }
    if(detected_mapped_plane_pairs[2].first.id != detected_mapped_plane_pairs[2].second.id) {
      if(!MapperUtils::check_plane_ids(yplane1_edges, detected_mapped_plane_pairs[2].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[2].first.plane_node, detected_mapped_plane_pairs[2].second.plane_node, information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        std::cout << "Adding new y1 plane " << std::endl;
      }
    }
    if(detected_mapped_plane_pairs[3].first.id != detected_mapped_plane_pairs[3].second.id) {
      if(!MapperUtils::check_plane_ids(yplane2_edges, detected_mapped_plane_pairs[3].second.plane_node)) {
        auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[3].first.plane_node, detected_mapped_plane_pairs[3].second.plane_node, information_2planes);
        graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
        std::cout << "Adding new y2 plane " << std::endl;
      }
    }
  }

  // std::cout << "found xplane1 id : " << (*found_x_plane1).id << std::endl;
  // std::cout << "found xplane1 coeffs : " << (*found_x_plane1).plane_node->estimate().coeffs() << std::endl;
  // std::cout << "found xplane2 id : " << (*found_x_plane2).id << std::endl;
  // std::cout << "found xplane2 coeffs : " << (*found_x_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "found yplane1 id : " << (*found_y_plane1).id << std::endl;
  // std::cout << "found yplane1 coeffs : " << (*found_y_plane1).plane_node->estimate().coeffs() << std::endl;
  // std::cout << "found yplane2 id : " << (*found_y_plane2).id << std::endl;
  // std::cout << "found yplane2 coeffs : " << (*found_y_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "mapped xplane1 id : " << (*found_mapped_x_plane1).id << std::endl;
  // std::cout << "mapped xplane1 coeffs : " << (*found_mapped_x_plane1).plane_node->estimate().coeffs() << std::endl;
  // std::cout << "mapped xplane2 id : " << (*found_mapped_x_plane2).id << std::endl;
  // std::cout << "mapped xplane2 coeffs : " << (*found_mapped_x_plane2).plane_node->estimate().coeffs() << std::endl;

  // std::cout << "mapped yplane1 id : " << (*found_mapped_y_plane1).id << std::endl;
  // std::cout << "mapped yplane1 coeffs : " << (*found_mapped_y_plane1).plane_node->estimate().coeffs() << std::endl;
  // std::cout << "mapped yplane2 id : " << (*found_mapped_y_plane2).id << std::endl;
  // std::cout << "mapped yplane2 coeffs : " << (*found_mapped_y_plane2).plane_node->estimate().coeffs() << std::endl;
}

std::pair<int, int> FiniteRoomMapper::associate_rooms(const Eigen::Vector2d& room_pose, const std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2, std::vector<std::pair<VerticalPlanes, VerticalPlanes>>& detected_mapped_plane_pairs) {
  float min_dist = 100;
  std::pair<int, int> data_association;
  data_association.first = -1;
  bool x_plane1_min_segment = false, x_plane2_min_segment = false;
  bool y_plane1_min_segment = false, y_plane2_min_segment = false;

  for(int i = 0; i < rooms_vec.size(); ++i) {
    float diff_x = room_pose(0) - rooms_vec[i].node->estimate()(0);
    float diff_y = room_pose(1) - rooms_vec[i].node->estimate()(1);
    float dist = sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
    ROS_DEBUG_NAMED("room planes", "dist room %f", dist);

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> current_detected_mapped_plane_pairs;
    auto found_mapped_xplane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_x1_id);
    auto found_mapped_xplane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_x2_id);
    std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
    std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;

    if(x_plane1.id == (*found_mapped_xplane1).id || x_plane1.id == (*found_mapped_xplane2).id) {
      x_plane1_min_segment = true;
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = x_plane1;
    } else if((x_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_xplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      x_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane1).cloud_seg_map, x_plane1.cloud_seg_map);
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = (*found_mapped_xplane1);
    } else {
      x_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane2).cloud_seg_map, x_plane1.cloud_seg_map);
      x1_detected_mapped_plane_pair.first = x_plane1;
      x1_detected_mapped_plane_pair.second = (*found_mapped_xplane2);
    }

    current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

    if(x_plane2.id == (*found_mapped_xplane1).id || x_plane2.id == (*found_mapped_xplane2).id) {
      x_plane2_min_segment = true;
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = x_plane2;
    } else if((x_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_xplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      x_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane1).cloud_seg_map, x_plane2.cloud_seg_map);
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = (*found_mapped_xplane1);
    } else {
      x_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane2).cloud_seg_map, x_plane2.cloud_seg_map);
      x2_detected_mapped_plane_pair.first = x_plane2;
      x2_detected_mapped_plane_pair.second = (*found_mapped_xplane2);
    }
    current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

    auto found_mapped_yplane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_y1_id);
    auto found_mapped_yplane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_y2_id);
    std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
    std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;

    if(y_plane1.id == (*found_mapped_yplane1).id || y_plane1.id == (*found_mapped_yplane2).id) {
      y_plane1_min_segment = true;
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = y_plane1;
    } else if((y_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_yplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      y_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane1).cloud_seg_map, y_plane1.cloud_seg_map);
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = (*found_mapped_yplane1);
    } else {
      y_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane2).cloud_seg_map, y_plane1.cloud_seg_map);
      y1_detected_mapped_plane_pair.first = y_plane1;
      y1_detected_mapped_plane_pair.second = (*found_mapped_yplane2);
    }
    current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

    if(y_plane2.id == (*found_mapped_yplane1).id || y_plane2.id == (*found_mapped_yplane2).id) {
      y_plane2_min_segment = true;
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = y_plane2;
    } else if((y_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_yplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      y_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane1).cloud_seg_map, y_plane2.cloud_seg_map);
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = (*found_mapped_yplane1);
    } else {
      y_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane2).cloud_seg_map, y_plane2.cloud_seg_map);
      y2_detected_mapped_plane_pair.first = y_plane2;
      y2_detected_mapped_plane_pair.second = (*found_mapped_yplane2);
    }
    current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

    if(dist < min_dist && (x_plane1_min_segment && x_plane2_min_segment && y_plane1_min_segment && y_plane2_min_segment)) {
      min_dist = dist;
      data_association.first = rooms_vec[i].id;
      data_association.second = i;
      detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
    }
  }

  ROS_DEBUG_NAMED("room planes", "min dist room %f", min_dist);
  if(min_dist > room_dist_threshold) data_association.first = -1;

  return data_association;
}

double FiniteRoomMapper::room_measurement(const int& plane_type, const Eigen::Vector2d& room_pose, const Eigen::Vector4d& plane) {
  double meas;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(room_pose(0)) > fabs(plane(3))) {
      meas = room_pose(0) - plane(3);
    } else {
      meas = plane(3) - room_pose(0);
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(room_pose(1)) > fabs(plane(3))) {
      meas = room_pose(1) - plane(3);
    } else {
      meas = plane(3) - room_pose(1);
    }
  }
  return meas;
}

bool FiniteRoomMapper::check_room_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexRoomXYLB* room_node) {
  for(auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom4Planes* edge_room_4planes = dynamic_cast<g2o::EdgeRoom4Planes*>(*edge_itr);
      if(edge_room_4planes) {
        g2o::VertexRoomXYLB* found_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_4planes->vertices()[0]);
        if(found_room_node->id() == room_node->id()) return true;
      }
    }

    if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom4Planes* edge_room_4planes = dynamic_cast<g2o::EdgeRoom4Planes*>(*edge_itr);
      if(edge_room_4planes) {
        g2o::VertexRoomXYLB* found_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_4planes->vertices()[0]);
        if(found_room_node->id() == room_node->id()) return true;
      }
    }
  }

  return false;
}

void FiniteRoomMapper::map_room_from_existing_infinite_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::InfiniteRooms& matched_x_infinite_room, const s_graphs::InfiniteRooms& matched_y_infinite_room, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped x and y infinite_rooms at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    det_room.plane_x1 = matched_x_infinite_room.plane1;
    det_room.plane_x2 = matched_x_infinite_room.plane2;
    det_room.plane_y1 = matched_y_infinite_room.plane1;
    det_room.plane_y2 = matched_y_infinite_room.plane2;
    det_room.plane_x1_id = matched_x_infinite_room.plane1_id;
    det_room.plane_x2_id = matched_x_infinite_room.plane2_id;
    det_room.plane_y1_id = matched_y_infinite_room.plane1_id;
    det_room.plane_y2_id = matched_y_infinite_room.plane2_id;
    det_room.connected_id = det_room_data.id;
    det_room.connected_neighbour_ids = det_room_data.neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);
    return;
  } else
    return;
}

void FiniteRoomMapper::map_room_from_existing_x_infinite_room(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::InfiniteRooms& matched_x_infinite_room, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped x infinite_rooms planes at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    det_room.plane_x1 = matched_x_infinite_room.plane1;
    det_room.plane_x2 = matched_x_infinite_room.plane2;
    Eigen::Vector4d y_plane1(det_room_data.y_planes[0].nx, det_room_data.y_planes[0].ny, det_room_data.y_planes[0].nz, det_room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(det_room_data.y_planes[1].nx, det_room_data.y_planes[1].ny, det_room_data.y_planes[1].nz, det_room_data.y_planes[1].d);
    det_room.plane_y1 = y_plane1;
    det_room.plane_y2 = y_plane2;
    det_room.plane_x1_id = matched_x_infinite_room.plane1_id;
    det_room.plane_x2_id = matched_x_infinite_room.plane2_id;
    det_room.plane_y1_id = det_room_data.y_planes[0].id;
    det_room.plane_y2_id = det_room_data.y_planes[1].id;
    det_room.connected_id = det_room_data.id;
    det_room.connected_neighbour_ids = det_room_data.neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);
    return;
  } else
    return;
}

void FiniteRoomMapper::map_room_from_existing_y_infinite_room(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::InfiniteRooms& matched_y_infinite_room, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped y infinite_rooms planes at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    Eigen::Vector4d x_plane1(det_room_data.x_planes[0].nx, det_room_data.x_planes[0].ny, det_room_data.x_planes[0].nz, det_room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(det_room_data.x_planes[1].nx, det_room_data.x_planes[1].ny, det_room_data.x_planes[1].nz, det_room_data.x_planes[1].d);
    det_room.plane_x1 = x_plane1;
    det_room.plane_x2 = x_plane2;
    det_room.plane_y1 = matched_y_infinite_room.plane1;
    det_room.plane_y2 = matched_y_infinite_room.plane2;
    det_room.plane_x1_id = det_room_data.x_planes[0].id;
    det_room.plane_x2_id = det_room_data.x_planes[1].id;
    det_room.plane_y1_id = matched_y_infinite_room.plane1_id;
    det_room.plane_y2_id = matched_y_infinite_room.plane2_id;
    det_room.connected_id = det_room_data.id;
    det_room.connected_neighbour_ids = det_room_data.neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);
    return;
  } else
    return;
}

void FiniteRoomMapper::remove_mapped_infinite_room(const int plane_type, std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms matched_infinite_room, std::vector<InfiniteRooms>& x_infinite_rooms, std::vector<InfiniteRooms>& y_infinite_rooms) {
  std::set<g2o::HyperGraph::Edge*> edges = matched_infinite_room.node->edges();
  for(auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
    g2o::EdgeRoom2Planes* edge_room_2planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
    if(edge_room_2planes) {
      if(graph_slam->remove_room_2planes_edge(edge_room_2planes)) std::cout << "removed edge - room-2planes " << std::endl;
      continue;
    }
  }

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = std::find_if(x_infinite_rooms.begin(), x_infinite_rooms.end(), boost::bind(&InfiniteRooms::id, _1) == matched_infinite_room.id);
      x_infinite_rooms.erase(mapped_infinite_room);
      std::cout << "removed overlapped x-infinite_room " << std::endl;
    }
  } else if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(graph_slam->remove_room_node(matched_infinite_room.node)) {
      auto mapped_infinite_room = std::find_if(y_infinite_rooms.begin(), y_infinite_rooms.end(), boost::bind(&InfiniteRooms::id, _1) == matched_infinite_room.id);
      y_infinite_rooms.erase(mapped_infinite_room);
      std::cout << "removed overlapped y-infinite_room " << std::endl;
    }
  }
}

}  // namespace s_graphs
