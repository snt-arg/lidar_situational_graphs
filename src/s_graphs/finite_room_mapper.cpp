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

void FiniteRoomMapper::lookup_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, const std::vector<Corridors>& x_corridors, const std::vector<Corridors>& y_corridors, std::vector<Rooms>& rooms_vec) {
  float min_dist_room_x_corr = 100;
  s_graphs::Corridors matched_x_corridor;
  for(const auto& current_x_corridor : x_corridors) {
    if((room_data.x_planes[0].id == current_x_corridor.plane1_id || room_data.x_planes[0].id == current_x_corridor.plane2_id) && (room_data.x_planes[1].id == current_x_corridor.plane1_id || room_data.x_planes[1].id == current_x_corridor.plane2_id)) {
      min_dist_room_x_corr = 0;
      matched_x_corridor = current_x_corridor;
      break;
    }
    float dist_room_x_corr = sqrt(pow(room_data.room_center.x - current_x_corridor.node->estimate()(0), 2) + pow(room_data.room_center.y - current_x_corridor.node->estimate()(1), 2));
    if(dist_room_x_corr < min_dist_room_x_corr) {
      min_dist_room_x_corr = dist_room_x_corr;
      matched_x_corridor = current_x_corridor;
    }
  }

  float min_dist_room_y_corr = 100;
  s_graphs::Corridors matched_y_corridor;
  for(const auto& current_y_corridor : y_corridors) {
    if((room_data.y_planes[0].id == current_y_corridor.plane1_id || room_data.y_planes[0].id == current_y_corridor.plane2_id) && (room_data.y_planes[1].id == current_y_corridor.plane1_id || room_data.y_planes[1].id == current_y_corridor.plane2_id)) {
      min_dist_room_y_corr = 0;
      matched_y_corridor = current_y_corridor;
      break;
    }

    float dist_room_y_corr = sqrt(pow(room_data.room_center.x - current_y_corridor.node->estimate()(0), 2) + pow(room_data.room_center.y - current_y_corridor.node->estimate()(1), 2));
    if(dist_room_y_corr < min_dist_room_y_corr) {
      min_dist_room_y_corr = dist_room_y_corr;
      matched_y_corridor = current_y_corridor;
    }
  }

  auto found_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[0].id);
  auto found_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[1].id);
  auto found_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[0].id);
  auto found_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[1].id);

  if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr < 1.0) {
    std::cout << "Adding a room using mapped x and y corridor planes " << std::endl;
    map_room_from_existing_corridors(graph_slam, room_data, matched_x_corridor, matched_y_corridor, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
  } else if(min_dist_room_x_corr < 1.0 && min_dist_room_y_corr > 1.0) {
    map_room_from_existing_x_corridor(graph_slam, room_data, matched_x_corridor, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    std::cout << "Will add room using mapped x corridor planes " << std::endl;
  } else if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr > 1.0) {
    std::cout << "Will add room using mapped y corridor planes " << std::endl;
    map_room_from_existing_y_corridor(graph_slam, room_data, matched_y_corridor, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
  }

  Eigen::Vector4d x_plane1(room_data.x_planes[0].nx, room_data.x_planes[0].ny, room_data.x_planes[0].nz, room_data.x_planes[0].d);
  Eigen::Vector4d x_plane2(room_data.x_planes[1].nx, room_data.x_planes[1].ny, room_data.x_planes[1].nz, room_data.x_planes[1].d);
  Eigen::Vector4d y_plane1(room_data.y_planes[0].nx, room_data.y_planes[0].ny, room_data.y_planes[0].nz, room_data.y_planes[0].d);
  Eigen::Vector4d y_plane2(room_data.y_planes[1].nx, room_data.y_planes[1].ny, room_data.y_planes[1].nz, room_data.y_planes[1].d);

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

  Eigen::Matrix2d information_room_plane;
  information_room_plane(0, 0) = room_information;
  information_room_plane(1, 1) = room_information;

  auto found_x_plane1 = x_vert_planes.begin();
  auto found_x_plane2 = x_vert_planes.begin();
  auto found_y_plane1 = y_vert_planes.begin();
  auto found_y_plane2 = y_vert_planes.begin();
  auto found_mapped_x_plane1 = x_vert_planes.begin();
  auto found_mapped_x_plane2 = x_vert_planes.begin();
  auto found_mapped_y_plane1 = y_vert_planes.begin();
  auto found_mapped_y_plane2 = y_vert_planes.begin();
  Eigen::Vector2d x_plane1_meas, x_plane2_meas;
  Eigen::Vector2d y_plane1_meas, y_plane2_meas;

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

  Eigen::Vector2d room_pose = compute_room_pose(x_room_pair_vec, y_room_pair_vec);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane2));
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "found room with pose " << room_pose << std::endl;
    room_data_association.first = graph_slam->num_vertices_local();
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

    x_plane1_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, x_room_pair_vec[0].plane_unflipped.coeffs());
    x_plane2_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, x_room_pair_vec[1].plane_unflipped.coeffs());

    y_plane1_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, y_room_pair_vec[0].plane_unflipped.coeffs());
    y_plane2_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, y_room_pair_vec[1].plane_unflipped.coeffs());

    /* Add parallel and perpendicular constraints here */
    if(use_parallel_plane_constraint) {
      MapperUtils::parallel_plane_constraint(graph_slam, (*found_x_plane1).plane_node, (*found_x_plane2).plane_node);
      MapperUtils::parallel_plane_constraint(graph_slam, (*found_y_plane1).plane_node, (*found_y_plane2).plane_node);
    }
    if(use_perpendicular_plane_constraint) {
      MapperUtils::perpendicular_plane_constraint(graph_slam, (*found_x_plane1).plane_node, (*found_y_plane1).plane_node);
      MapperUtils::perpendicular_plane_constraint(graph_slam, (*found_x_plane1).plane_node, (*found_y_plane2).plane_node);
      MapperUtils::perpendicular_plane_constraint(graph_slam, (*found_x_plane2).plane_node, (*found_y_plane1).plane_node);
      MapperUtils::perpendicular_plane_constraint(graph_slam, (*found_x_plane2).plane_node, (*found_y_plane2).plane_node);
    }

    auto edge_x_plane1 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane1).plane_node, x_plane1_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_x_plane1, "Huber", 1.0);

    auto edge_x_plane2 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane2).plane_node, x_plane2_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_x_plane2, "Huber", 1.0);

    auto edge_y_plane1 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane1).plane_node, y_plane1_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_y_plane1, "Huber", 1.0);

    auto edge_y_plane2 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane2).plane_node, y_plane2_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_y_plane2, "Huber", 1.0);

  } else {
    /* add the edge between detected planes and the corridor */
    room_node = rooms_vec[room_data_association.second].node;
    std::cout << "Matched det room with pose " << room_pose << " to mapped room with id " << room_data_association.first << " and pose " << room_node->estimate() << std::endl;

    rooms_vec[room_data_association.second].connected_id = x_room_pair_vec[0].connected_id;
    for(const auto& det_room_neighbour : x_room_pair_vec[0].connected_neighbour_ids) {
      rooms_vec[room_data_association.second].connected_neighbour_ids.push_back(det_room_neighbour);
    }

    found_mapped_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_x1_id);
    found_mapped_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_x2_id);
    Eigen::Vector4d found_mapped_x_plane1_coeffs, found_mapped_x_plane2_coeffs;
    found_mapped_x_plane1_coeffs = (*found_mapped_x_plane1).plane_node->estimate().coeffs();
    found_mapped_x_plane2_coeffs = (*found_mapped_x_plane2).plane_node->estimate().coeffs();
    plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_x_plane1_coeffs, (*found_mapped_x_plane1).cloud_seg_map->points.back().x, (*found_mapped_x_plane1).cloud_seg_map->points.back().y);
    plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_x_plane2_coeffs, (*found_mapped_x_plane2).cloud_seg_map->points.back().x, (*found_mapped_x_plane2).cloud_seg_map->points.back().y);

    bool found_new_x_plane = false;
    if((*found_x_plane1).id == (*found_mapped_x_plane1).id)
      x_plane1_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane1_coeffs);
    else if((*found_x_plane1).id == (*found_mapped_x_plane2).id)
      x_plane1_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane2_coeffs);
    else {
      std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
      if((*found_x_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_x_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        x_plane1_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane1_coeffs);
        dupl_plane_pair = std::make_pair(*found_x_plane1, *found_mapped_x_plane1);
      } else {
        x_plane1_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane2_coeffs);
        dupl_plane_pair = std::make_pair(*found_x_plane1, *found_mapped_x_plane2);
      }
      found_new_x_plane = true;
      dupl_x_vert_planes.push_back(dupl_plane_pair);

      std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_x_plane1).plane_node->edges();
      if(!check_room_ids(PlaneUtils::plane_class::X_VERT_PLANE, plane_edges, room_node)) {
        std::cout << "adding edge between xplane1 and room node" << std::endl;
        auto edge_x_plane1 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane1).plane_node, x_plane1_meas, information_room_plane);
        graph_slam->add_robust_kernel(edge_x_plane1, "Huber", 1.0);
      }
    }

    if((*found_x_plane2).id == (*found_mapped_x_plane1).id)
      x_plane2_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane1_coeffs);
    else if((*found_x_plane2).id == (*found_mapped_x_plane2).id)
      x_plane2_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane2_coeffs);
    else {
      std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
      if((*found_x_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_x_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        x_plane2_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane1_coeffs);
        dupl_plane_pair = std::make_pair(*found_x_plane2, *found_mapped_x_plane1);
      } else {
        x_plane2_meas = room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane2_coeffs);
        dupl_plane_pair = std::make_pair(*found_x_plane2, *found_mapped_x_plane2);
      }
      found_new_x_plane = true;
      dupl_x_vert_planes.push_back(dupl_plane_pair);

      std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_x_plane2).plane_node->edges();
      if(!check_room_ids(PlaneUtils::plane_class::X_VERT_PLANE, plane_edges, room_node)) {
        std::cout << "adding edge between xplane2 and room node" << std::endl;
        auto edge_x_plane2 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane2).plane_node, x_plane2_meas, information_room_plane);
        graph_slam->add_robust_kernel(edge_x_plane2, "Huber", 1.0);
      }
    }

    if(use_parallel_plane_constraint && found_new_x_plane) {
      MapperUtils::parallel_plane_constraint(graph_slam, (*found_x_plane1).plane_node, (*found_x_plane2).plane_node);
    }

    found_mapped_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_y1_id);
    found_mapped_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_y2_id);
    Eigen::Vector4d found_mapped_y_plane1_coeffs, found_mapped_y_plane2_coeffs;
    found_mapped_y_plane1_coeffs = (*found_mapped_y_plane1).plane_node->estimate().coeffs();
    found_mapped_y_plane2_coeffs = (*found_mapped_y_plane2).plane_node->estimate().coeffs();
    plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_y_plane1_coeffs, (*found_mapped_y_plane1).cloud_seg_map->points.back().x, (*found_mapped_y_plane1).cloud_seg_map->points.back().y);
    plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_y_plane2_coeffs, (*found_mapped_y_plane2).cloud_seg_map->points.back().x, (*found_mapped_y_plane2).cloud_seg_map->points.back().y);

    bool found_new_y_plane = false;
    if((*found_y_plane1).id == (*found_mapped_y_plane1).id)
      y_plane1_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane1_coeffs);
    else if((*found_y_plane1).id == (*found_mapped_y_plane2).id)
      y_plane1_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane2_coeffs);
    else {
      std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
      if((*found_y_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_y_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        y_plane1_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane1_coeffs);
        dupl_plane_pair = std::make_pair(*found_y_plane1, *found_mapped_y_plane1);
      } else {
        y_plane1_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane2_coeffs);
        dupl_plane_pair = std::make_pair(*found_y_plane1, *found_mapped_y_plane2);
      }
      found_new_y_plane = true;
      dupl_y_vert_planes.push_back(dupl_plane_pair);

      std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_y_plane1).plane_node->edges();
      if(!check_room_ids(PlaneUtils::plane_class::Y_VERT_PLANE, plane_edges, room_node)) {
        std::cout << "adding edge between yplane1 and room node" << std::endl;
        auto edge_y_plane1 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane1).plane_node, y_plane1_meas, information_room_plane);
        graph_slam->add_robust_kernel(edge_y_plane1, "Huber", 1.0);
      }
    }

    if((*found_y_plane2).id == (*found_mapped_y_plane1).id)
      y_plane2_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane1_coeffs);
    else if((*found_y_plane2).id == (*found_mapped_y_plane2).id)
      y_plane2_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane2_coeffs);
    else {
      std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
      if((*found_y_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_y_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        y_plane2_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane1_coeffs);
        dupl_plane_pair = std::make_pair(*found_y_plane2, *found_mapped_y_plane1);
      } else {
        y_plane2_meas = room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane2_coeffs);
        dupl_plane_pair = std::make_pair(*found_y_plane2, *found_mapped_y_plane2);
      }
      found_new_y_plane = true;
      dupl_y_vert_planes.push_back(dupl_plane_pair);

      std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_y_plane2).plane_node->edges();
      if(!check_room_ids(PlaneUtils::plane_class::Y_VERT_PLANE, plane_edges, room_node)) {
        auto edge_y_plane2 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane2).plane_node, y_plane2_meas, information_room_plane);
        graph_slam->add_robust_kernel(edge_y_plane2, "Huber", 1.0);
      }
    }

    if(use_parallel_plane_constraint && found_new_y_plane) {
      MapperUtils::parallel_plane_constraint(graph_slam, (*found_y_plane1).plane_node, (*found_y_plane2).plane_node);
    }
  }

  std::cout << "found xplane1 id : " << (*found_x_plane1).id << std::endl;
  std::cout << "found xplane1 coeffs : " << (*found_x_plane1).plane_node->estimate().coeffs() << std::endl;
  std::cout << "found xplane2 id : " << (*found_x_plane2).id << std::endl;
  std::cout << "found xplane2 coeffs : " << (*found_x_plane2).plane_node->estimate().coeffs() << std::endl;

  std::cout << "found yplane1 id : " << (*found_y_plane1).id << std::endl;
  std::cout << "found yplane1 coeffs : " << (*found_y_plane1).plane_node->estimate().coeffs() << std::endl;
  std::cout << "found yplane2 id : " << (*found_y_plane2).id << std::endl;
  std::cout << "found yplane2 coeffs : " << (*found_y_plane2).plane_node->estimate().coeffs() << std::endl;

  std::cout << "mapped xplane1 id : " << (*found_mapped_x_plane1).id << std::endl;
  std::cout << "mapped xplane1 coeffs : " << (*found_mapped_x_plane1).plane_node->estimate().coeffs() << std::endl;
  std::cout << "mapped xplane2 id : " << (*found_mapped_x_plane2).id << std::endl;
  std::cout << "mapped xplane2 coeffs : " << (*found_mapped_x_plane2).plane_node->estimate().coeffs() << std::endl;

  std::cout << "mapped yplane1 id : " << (*found_mapped_y_plane1).id << std::endl;
  std::cout << "mapped yplane1 coeffs : " << (*found_mapped_y_plane1).plane_node->estimate().coeffs() << std::endl;
  std::cout << "mapped yplane2 id : " << (*found_mapped_y_plane2).id << std::endl;
  std::cout << "mapped yplane2 coeffs : " << (*found_mapped_y_plane2).plane_node->estimate().coeffs() << std::endl;
}

/*TODO:HB Move this to plane_utils.hpp */
Eigen::Vector2d FiniteRoomMapper::compute_room_pose(const std::vector<plane_data_list>& x_room_pair_vec, const std::vector<plane_data_list>& y_room_pair_vec) {
  Eigen::Vector2d room_pose(0, 0);
  Eigen::Vector3d vec_x, vec_y;
  Eigen::Vector4d x_plane1 = x_room_pair_vec[0].plane_unflipped.coeffs(), x_plane2 = x_room_pair_vec[1].plane_unflipped.coeffs();
  Eigen::Vector4d y_plane1 = y_room_pair_vec[0].plane_unflipped.coeffs(), y_plane2 = y_room_pair_vec[1].plane_unflipped.coeffs();

  if(fabs(x_plane1(3)) > fabs(x_plane2(3))) {
    vec_x = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) - fabs(x_plane2(3)) * x_plane2.head(3))) + fabs(x_plane2(3)) * x_plane2.head(3);
  } else {
    vec_x = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) - fabs(x_plane1(3)) * x_plane1.head(3))) + fabs(x_plane1(3)) * x_plane1.head(3);
  }

  if(fabs(y_plane1(3)) > fabs(y_plane2(3))) {
    vec_y = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) - fabs(y_plane2(3)) * y_plane2.head(3))) + fabs(y_plane2(3)) * y_plane2.head(3);
  } else {
    vec_y = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) - fabs(y_plane1(3)) * y_plane1.head(3))) + fabs(y_plane1(3)) * y_plane1.head(3);
  }

  Eigen::Vector3d final_vec = vec_x + vec_y;
  room_pose(0) = final_vec(0);
  room_pose(1) = final_vec(1);

  return room_pose;
}

std::pair<int, int> FiniteRoomMapper::associate_rooms(const Eigen::Vector2d& room_pose, const std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
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

    auto found_mapped_xplane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_x1_id);
    auto found_mapped_xplane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_x2_id);

    if(x_plane1.id == (*found_mapped_xplane1).id || x_plane1.id == (*found_mapped_xplane2).id) {
      x_plane1_min_segment = true;
    } else if((x_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_xplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      x_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane1).cloud_seg_map, x_plane1.cloud_seg_map);
    } else {
      x_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane2).cloud_seg_map, x_plane1.cloud_seg_map);
    }

    if(x_plane2.id == (*found_mapped_xplane1).id || x_plane2.id == (*found_mapped_xplane2).id) {
      x_plane2_min_segment = true;
    } else if((x_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_xplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      x_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane1).cloud_seg_map, x_plane2.cloud_seg_map);
    } else {
      x_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_xplane2).cloud_seg_map, x_plane2.cloud_seg_map);
    }

    auto found_mapped_yplane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_y1_id);
    auto found_mapped_yplane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[i].plane_y2_id);

    if(y_plane1.id == (*found_mapped_yplane1).id || y_plane1.id == (*found_mapped_yplane2).id) {
      y_plane1_min_segment = true;
    } else if((y_plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_yplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      y_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane1).cloud_seg_map, y_plane1.cloud_seg_map);
    } else {
      y_plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane2).cloud_seg_map, y_plane1.cloud_seg_map);
    }

    if(y_plane2.id == (*found_mapped_yplane1).id || y_plane2.id == (*found_mapped_yplane2).id) {
      y_plane2_min_segment = true;
    } else if((y_plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_yplane1).plane_node->estimate().coeffs().head(3)) > 0) {
      y_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane1).cloud_seg_map, y_plane2.cloud_seg_map);
    } else {
      y_plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_yplane2).cloud_seg_map, y_plane2.cloud_seg_map);
    }

    if(dist < min_dist && (x_plane1_min_segment && x_plane2_min_segment && y_plane1_min_segment && y_plane2_min_segment)) {
      min_dist = dist;
      data_association.first = rooms_vec[i].id;
      data_association.second = i;
    }
  }

  ROS_DEBUG_NAMED("room planes", "min dist room %f", min_dist);
  if(min_dist > room_dist_threshold) data_association.first = -1;

  return data_association;
}

Eigen::Vector2d FiniteRoomMapper::room_measurement(const int& plane_type, const Eigen::Vector2d& room_pose, const Eigen::Vector4d& plane) {
  Eigen::Vector2d meas;
  Eigen::Vector2d room_pose_transformed = room_pose.dot(plane.head(2)) * plane.head(2);
  Eigen::Vector2d plane_vec = fabs(plane(3)) * plane.head(2);

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(room_pose(0)) > fabs(plane(3))) {
      meas = room_pose_transformed - plane_vec;
    } else {
      meas = plane_vec - room_pose_transformed;
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(room_pose(1)) > fabs(plane(3))) {
      meas = room_pose_transformed - plane_vec;
    } else {
      meas = plane_vec - room_pose_transformed;
    }
  }

  return meas;
}

bool FiniteRoomMapper::check_room_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexRoomXYLB* room_node) {
  for(auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoomXPlane* edge_room_xplane = dynamic_cast<g2o::EdgeRoomXPlane*>(*edge_itr);
      if(edge_room_xplane) {
        g2o::VertexRoomXYLB* found_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_xplane->vertices()[0]);
        if(found_room_node->id() == room_node->id()) return true;
      }
    }

    if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoomYPlane* edge_room_yplane = dynamic_cast<g2o::EdgeRoomYPlane*>(*edge_itr);
      if(edge_room_yplane) {
        g2o::VertexRoomXYLB* found_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_yplane->vertices()[0]);
        if(found_room_node->id() == room_node->id()) return true;
      }
    }
  }

  return false;
}

void FiniteRoomMapper::map_room_from_existing_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_x_corridor, const s_graphs::Corridors& matched_y_corridor, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped x and y corridors at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->num_vertices_local();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    det_room.plane_x1 = matched_x_corridor.plane1;
    det_room.plane_x2 = matched_x_corridor.plane2;
    det_room.plane_y1 = matched_y_corridor.plane1;
    det_room.plane_y2 = matched_y_corridor.plane2;
    det_room.plane_x1_id = matched_x_corridor.plane1_id;
    det_room.plane_x2_id = matched_x_corridor.plane2_id;
    det_room.plane_y1_id = matched_y_corridor.plane1_id;
    det_room.plane_y2_id = matched_y_corridor.plane2_id;
    det_room.connected_id = det_room_data.id;
    det_room.connected_neighbour_ids = det_room_data.neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);
    return;
  } else
    return;
}

void FiniteRoomMapper::map_room_from_existing_x_corridor(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_x_corridor, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped x corridors planes at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->num_vertices_local();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    det_room.plane_x1 = matched_x_corridor.plane1;
    det_room.plane_x2 = matched_x_corridor.plane2;
    Eigen::Vector4d y_plane1(det_room_data.y_planes[0].nx, det_room_data.y_planes[0].ny, det_room_data.y_planes[0].nz, det_room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(det_room_data.y_planes[1].nx, det_room_data.y_planes[1].ny, det_room_data.y_planes[1].nz, det_room_data.y_planes[1].d);
    det_room.plane_y1 = y_plane1;
    det_room.plane_y2 = y_plane2;
    det_room.plane_x1_id = matched_x_corridor.plane1_id;
    det_room.plane_x2_id = matched_x_corridor.plane2_id;
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

void FiniteRoomMapper::map_room_from_existing_y_corridor(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_y_corridor, std::vector<Rooms>& rooms_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  g2o::VertexRoomXYLB* room_node;
  std::pair<int, int> room_data_association;

  Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2);
  if((rooms_vec.empty() || room_data_association.first == -1)) {
    std::cout << "Add a room using mapped y corridors planes at pose" << room_pose << std::endl;
    room_data_association.first = graph_slam->num_vertices_local();
    room_node = graph_slam->add_room_node(room_pose);
    Rooms det_room;
    det_room.id = room_data_association.first;
    Eigen::Vector4d x_plane1(det_room_data.x_planes[0].nx, det_room_data.x_planes[0].ny, det_room_data.x_planes[0].nz, det_room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(det_room_data.x_planes[1].nx, det_room_data.x_planes[1].ny, det_room_data.x_planes[1].nz, det_room_data.x_planes[1].d);
    det_room.plane_x1 = x_plane1;
    det_room.plane_x2 = x_plane2;
    det_room.plane_y1 = matched_y_corridor.plane1;
    det_room.plane_y2 = matched_y_corridor.plane2;
    det_room.plane_x1_id = det_room_data.x_planes[0].id;
    det_room.plane_x2_id = det_room_data.x_planes[1].id;
    det_room.plane_y1_id = matched_y_corridor.plane1_id;
    det_room.plane_y2_id = matched_y_corridor.plane2_id;
    det_room.connected_id = det_room_data.id;
    det_room.connected_neighbour_ids = det_room_data.neighbour_ids;
    det_room.node = room_node;
    rooms_vec.push_back(det_room);
    return;
  } else
    return;
}

}  // namespace s_graphs