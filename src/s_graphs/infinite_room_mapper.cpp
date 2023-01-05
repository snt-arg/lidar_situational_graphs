// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_mapper.hpp>

namespace s_graphs {

InfiniteRoomMapper::InfiniteRoomMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;

  infinite_room_information = node->get_parameter("infinite_room_information").get_parameter_value().get<double>();
  infinite_room_dist_threshold = node->get_parameter("infinite_room_dist_threshold").get_parameter_value().get<double>();
  dupl_plane_matching_information = node->get_parameter("dupl_plane_matching_information").get_parameter_value().get<double>();

  use_parallel_plane_constraint = node->get_parameter("use_parallel_plane_constraint").get_parameter_value().get<bool>();
  use_perpendicular_plane_constraint = node->get_parameter("use_perpendicular_plane_constraint").get_parameter_value().get<bool>();

  plane_utils.reset(new PlaneUtils());
}

InfiniteRoomMapper::~InfiniteRoomMapper() {}

void InfiniteRoomMapper::lookup_infinite_rooms(std::shared_ptr<GraphSLAM>& graph_slam, const int& plane_type, const s_graphs::msg::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<InfiniteRooms>& x_infinite_rooms, std::vector<InfiniteRooms>& y_infinite_rooms, const std::vector<Rooms>& rooms_vec) {
  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    // check the distance with the current room vector
    Rooms matched_room;
    float min_dist_x_corr_room = 100;
    for(const auto& current_room : rooms_vec) {
      if((room_data.x_planes[0].id == current_room.plane_x1_id || room_data.x_planes[0].id == current_room.plane_x2_id) && (room_data.x_planes[1].id == current_room.plane_x1_id || room_data.x_planes[1].id == current_room.plane_x2_id)) {
        min_dist_x_corr_room = 0;
        matched_room = current_room;
        break;
      }

      float dist_x_corr_room = sqrt(pow(room_data.room_center.x - current_room.node->estimate()(0), 2) + pow(room_data.room_center.y - current_room.node->estimate()(1), 2));
      if(dist_x_corr_room < min_dist_x_corr_room) {
        min_dist_x_corr_room = dist_x_corr_room;
        matched_room = current_room;
      }
    }

    if(min_dist_x_corr_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting an x infinite_room" << std::endl;
      return;
    }

    // factor the infinite_room here
    std::cout << "factoring x infinite_room" << std::endl;
    Eigen::Vector4d x_plane1(room_data.x_planes[0].nx, room_data.x_planes[0].ny, room_data.x_planes[0].nz, room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(room_data.x_planes[1].nx, room_data.x_planes[1].ny, room_data.x_planes[1].nz, room_data.x_planes[1].d);
    plane_data_list x_plane1_data, x_plane2_data;
    x_plane1_data.plane_id = room_data.x_planes[0].id;
    x_plane1_data.plane_unflipped = x_plane1;
    x_plane1_data.plane_centroid(0) = room_data.room_center.x;
    x_plane1_data.plane_centroid(1) = room_data.room_center.y;
    x_plane1_data.cluster_center(0) = room_data.cluster_center.x;
    x_plane1_data.cluster_center(1) = room_data.cluster_center.y;

    x_plane2_data.plane_id = room_data.x_planes[1].id;
    x_plane2_data.plane_unflipped = x_plane2;
    x_plane2_data.plane_centroid(0) = room_data.room_center.x;
    x_plane2_data.plane_centroid(1) = room_data.room_center.y;
    x_plane2_data.cluster_center(0) = room_data.cluster_center.x;
    x_plane2_data.cluster_center(1) = room_data.cluster_center.y;
    x_plane1_data.connected_id = room_data.id;

    // get the infinite_room neighbours
    for(const auto& room_neighbour_id : room_data.neighbour_ids) {
      x_plane1_data.connected_neighbour_ids.push_back(room_neighbour_id);
    }

    factor_infinite_rooms(graph_slam, PlaneUtils::plane_class::X_VERT_PLANE, x_plane1_data, x_plane2_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_infinite_rooms, y_infinite_rooms);
  }

  else if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    float min_dist_y_corr_room = 100;
    Rooms matched_room;
    for(const auto& current_room : rooms_vec) {
      if((room_data.y_planes[0].id == current_room.plane_y1_id || room_data.y_planes[0].id == current_room.plane_y2_id) && (room_data.y_planes[1].id == current_room.plane_y1_id || room_data.y_planes[1].id == current_room.plane_y2_id)) {
        min_dist_y_corr_room = 0;
        matched_room = current_room;
        break;
      }

      float dist_y_corr_room = sqrt(pow(room_data.room_center.x - current_room.node->estimate()(0), 2) + pow(room_data.room_center.y - current_room.node->estimate()(1), 2));
      if(dist_y_corr_room < min_dist_y_corr_room) {
        min_dist_y_corr_room = dist_y_corr_room;
        matched_room = current_room;
      }
    }
    if(min_dist_y_corr_room < 1.0) {
      std::cout << "Room already exists in the given location, not inserting a y infinite_room" << std::endl;
      return;
    }

    // factor the infinite_room here
    std::cout << "factoring y infinite_room" << std::endl;
    Eigen::Vector4d y_plane1(room_data.y_planes[0].nx, room_data.y_planes[0].ny, room_data.y_planes[0].nz, room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(room_data.y_planes[1].nx, room_data.y_planes[1].ny, room_data.y_planes[1].nz, room_data.y_planes[1].d);
    plane_data_list y_plane1_data, y_plane2_data;
    y_plane1_data.plane_id = room_data.y_planes[0].id;
    y_plane1_data.plane_unflipped = y_plane1;
    y_plane1_data.plane_centroid(0) = room_data.room_center.x;
    y_plane1_data.plane_centroid(1) = room_data.room_center.y;
    y_plane1_data.cluster_center(0) = room_data.cluster_center.x;
    y_plane1_data.cluster_center(1) = room_data.cluster_center.y;

    y_plane2_data.plane_id = room_data.y_planes[1].id;
    y_plane2_data.plane_unflipped = y_plane2;
    y_plane2_data.plane_centroid(0) = room_data.room_center.x;
    y_plane2_data.plane_centroid(1) = room_data.room_center.y;
    y_plane2_data.cluster_center(0) = room_data.cluster_center.x;
    y_plane2_data.cluster_center(1) = room_data.cluster_center.y;
    y_plane1_data.connected_id = room_data.id;

    // get the infinite_room neighbours
    for(const auto& room_neighbour_id : room_data.neighbour_ids) {
      y_plane1_data.connected_neighbour_ids.push_back(room_neighbour_id);
    }
    factor_infinite_rooms(graph_slam, PlaneUtils::plane_class::Y_VERT_PLANE, y_plane1_data, y_plane2_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_infinite_rooms, y_infinite_rooms);
  }
}

void InfiniteRoomMapper::factor_infinite_rooms(std::shared_ptr<GraphSLAM>& graph_slam, const int plane_type, const plane_data_list& corr_plane1_pair, const plane_data_list& corr_plane2_pair, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<InfiniteRooms>& x_infinite_rooms, std::vector<InfiniteRooms>& y_infinite_rooms) {
  g2o::VertexRoomXYLB* corr_node;
  g2o::VertexRoomXYLB* cluster_center_node;
  std::pair<int, int> corr_data_association;
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

  // Eigen::Vector2d corr_pose = compute_infinite_room_pose(plane_type, corr_plane1_pair.plane_centroid, corr_plane1_pair.plane_unflipped.coeffs(), corr_plane2_pair.plane_unflipped.coeffs());
  Eigen::Vector2d corr_pose(corr_plane1_pair.plane_centroid(0), corr_plane1_pair.plane_centroid(1));
  RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "final infinite_room plane 1 %f %f %f %f", corr_plane1_pair.plane_unflipped.coeffs()(0), corr_plane1_pair.plane_unflipped.coeffs()(1), corr_plane1_pair.plane_unflipped.coeffs()(2),
               corr_plane1_pair.plane_unflipped.coeffs()(3));
  RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "final infinite_room plane 2 %f %f %f %f", corr_plane2_pair.plane_unflipped.coeffs()(0), corr_plane2_pair.plane_unflipped.coeffs()(1), corr_plane2_pair.plane_unflipped.coeffs()(2),
               corr_plane2_pair.plane_unflipped.coeffs()(3));

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    auto found_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
    auto found_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);

    if(found_plane1 == x_vert_planes.end() || found_plane2 == x_vert_planes.end()) {
      std::cout << "did not find planes for x infinite_room " << std::endl;
      return;
    }

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    corr_data_association = associate_infinite_rooms(plane_type, corr_pose, (*found_plane1), (*found_plane2), x_vert_planes, y_vert_planes, x_infinite_rooms, y_infinite_rooms, detected_mapped_plane_pairs);

    if((x_infinite_rooms.empty() || corr_data_association.first == -1)) {
      std::cout << "found an X infinite_room with pre pose " << corr_pose << " between plane " << corr_plane1_pair.plane_unflipped.coeffs() << " and plane " << corr_plane2_pair.plane_unflipped.coeffs() << std::endl;

      corr_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      corr_node = graph_slam->add_room_node(corr_pose);
      cluster_center_node = graph_slam->add_room_node(corr_plane1_pair.cluster_center);
      cluster_center_node->setFixed(true);
      // graph_slam->add_room_yprior_edge(corr_node, corr_pose(1), information_infinite_room_prior);
      InfiniteRooms det_infinite_room;
      det_infinite_room.id = corr_data_association.first;
      det_infinite_room.plane1 = corr_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = corr_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = corr_plane1_pair.plane_id;
      det_infinite_room.plane2_id = corr_plane2_pair.plane_id;
      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = corr_node;
      det_infinite_room.connected_id = corr_plane1_pair.connected_id;
      det_infinite_room.connected_neighbour_ids = corr_plane1_pair.connected_neighbour_ids;
      x_infinite_rooms.push_back(det_infinite_room);

      auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, cluster_center_node, information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);

    } else {
      /* add the edge between detected planes and the infinite_room */
      corr_node = x_infinite_rooms[corr_data_association.second].node;
      std::cout << "Matched det infinite_room X with pre pose " << corr_pose << " to mapped infinite_room with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

      std::set<g2o::HyperGraph::Edge*> plane1_edges = (*found_plane1).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges = (*found_plane2).plane_node->edges();

      if(detected_mapped_plane_pairs[0].first.id != detected_mapped_plane_pairs[0].second.id) {
        if(!MapperUtils::check_plane_ids(plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[0].first.plane_node, detected_mapped_plane_pairs[0].second.plane_node, information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          std::cout << "Adding new x1 plane " << std::endl;
        }
      }
      if(detected_mapped_plane_pairs[1].first.id != detected_mapped_plane_pairs[1].second.id) {
        if(!MapperUtils::check_plane_ids(plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[1].first.plane_node, detected_mapped_plane_pairs[1].second.plane_node, information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          std::cout << "Adding new x2 plane " << std::endl;
        }
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
    auto found_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);

    if(found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end()) return;

    std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
    corr_data_association = associate_infinite_rooms(plane_type, corr_pose, (*found_plane1), (*found_plane2), x_vert_planes, y_vert_planes, x_infinite_rooms, y_infinite_rooms, detected_mapped_plane_pairs);

    if((y_infinite_rooms.empty() || corr_data_association.first == -1)) {
      std::cout << "found an Y infinite_room with pre pose " << corr_pose << " between plane " << corr_plane1_pair.plane_unflipped.coeffs() << " and plane " << corr_plane2_pair.plane_unflipped.coeffs() << std::endl;

      corr_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      corr_node = graph_slam->add_room_node(corr_pose);
      cluster_center_node = graph_slam->add_room_node(corr_plane1_pair.cluster_center);
      cluster_center_node->setFixed(true);
      // graph_slam->add_room_xprior_edge(corr_node, corr_pose(0), information_infinite_room_prior);

      InfiniteRooms det_infinite_room;
      det_infinite_room.id = corr_data_association.first;
      det_infinite_room.plane1 = corr_plane1_pair.plane_unflipped;
      det_infinite_room.plane2 = corr_plane2_pair.plane_unflipped;
      det_infinite_room.plane1_id = corr_plane1_pair.plane_id;
      det_infinite_room.plane2_id = corr_plane2_pair.plane_id;
      det_infinite_room.cluster_center_node = cluster_center_node;
      det_infinite_room.node = corr_node;
      det_infinite_room.connected_id = corr_plane1_pair.connected_id;
      det_infinite_room.connected_neighbour_ids = corr_plane1_pair.connected_neighbour_ids;
      y_infinite_rooms.push_back(det_infinite_room);

      auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, cluster_center_node, information_infinite_room_planes);
      graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);
    } else {
      /* add the edge between detected planes and the infinite_room */
      corr_node = y_infinite_rooms[corr_data_association.second].node;
      std::cout << "Matched det infinite_room Y with pre pose " << corr_pose << " to mapped infinite_room with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

      std::set<g2o::HyperGraph::Edge*> plane1_edges = (*found_plane1).plane_node->edges();
      std::set<g2o::HyperGraph::Edge*> plane2_edges = (*found_plane2).plane_node->edges();

      if(detected_mapped_plane_pairs[0].first.id != detected_mapped_plane_pairs[0].second.id) {
        if(!MapperUtils::check_plane_ids(plane1_edges, detected_mapped_plane_pairs[0].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[0].first.plane_node, detected_mapped_plane_pairs[0].second.plane_node, information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          std::cout << "Adding new y1 plane " << std::endl;
        }
      }
      if(detected_mapped_plane_pairs[1].first.id != detected_mapped_plane_pairs[1].second.id) {
        if(!MapperUtils::check_plane_ids(plane2_edges, detected_mapped_plane_pairs[1].second.plane_node)) {
          auto edge_planes = graph_slam->add_2planes_edge(detected_mapped_plane_pairs[1].first.plane_node, detected_mapped_plane_pairs[1].second.plane_node, information_2planes);
          graph_slam->add_robust_kernel(edge_planes, "Huber", 1.0);
          std::cout << "Adding new y2 plane " << std::endl;
        }
      }
    }
  }

  return;
}

std::pair<int, int> InfiniteRoomMapper::associate_infinite_rooms(const int& plane_type, const Eigen::Vector2d& corr_pose, const VerticalPlanes& plane1, const VerticalPlanes& plane2, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const std::vector<InfiniteRooms>& x_infinite_rooms, const std::vector<InfiniteRooms>& y_infinite_rooms, std::vector<std::pair<VerticalPlanes, VerticalPlanes>>& detected_mapped_plane_pairs) {
  float min_dist = 100;
  bool plane1_min_segment = false, plane2_min_segment = false;

  std::pair<int, int> data_association;
  data_association.first = -1;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for(int i = 0; i < x_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - x_infinite_rooms[i].node->estimate()(0), 2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>> current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> x1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> x2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_infinite_rooms[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_infinite_rooms[i].plane2_id);

      if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = plane1;
      } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);
        x1_detected_mapped_plane_pair.first = plane1;
        x1_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(x1_detected_mapped_plane_pair);

      if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = plane2;
      } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);
        x2_detected_mapped_plane_pair.first = plane2;
        x2_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(x2_detected_mapped_plane_pair);

      if(dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = x_infinite_rooms[i].id;
        data_association.second = i;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "dist x corr %f", dist);
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for(int i = 0; i < y_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(corr_pose(1) - y_infinite_rooms[i].node->estimate()(1), 2));

      std::vector<std::pair<VerticalPlanes, VerticalPlanes>> current_detected_mapped_plane_pairs;
      std::pair<VerticalPlanes, VerticalPlanes> y1_detected_mapped_plane_pair;
      std::pair<VerticalPlanes, VerticalPlanes> y2_detected_mapped_plane_pair;
      auto found_mapped_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_infinite_rooms[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_infinite_rooms[i].plane2_id);

      if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = plane1;
      } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);
        y1_detected_mapped_plane_pair.first = plane1;
        y1_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(y1_detected_mapped_plane_pair);

      if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = plane2;
      } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (*found_mapped_plane1);
      } else {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);
        y2_detected_mapped_plane_pair.first = plane2;
        y2_detected_mapped_plane_pair.second = (*found_mapped_plane2);
      }
      current_detected_mapped_plane_pairs.push_back(y2_detected_mapped_plane_pair);

      if(dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = y_infinite_rooms[i].id;
        data_association.second = i;
        detected_mapped_plane_pairs = current_detected_mapped_plane_pairs;
        RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "dist y corr %f", dist);
      }
    }
  }

  // RCLCPP_DEBUG(node_obj->get_logger(),"infinite_room planes", "min dist %f", min_dist);
  if(min_dist > infinite_room_dist_threshold) data_association.first = -1;

  return data_association;
}

std::pair<int, int> InfiniteRoomMapper::associate_infinite_rooms(const int& plane_type, const Eigen::Vector2d& corr_pose, const std::vector<InfiniteRooms>& x_infinite_rooms, const std::vector<InfiniteRooms>& y_infinite_rooms) {
  float min_dist = 100;
  std::pair<int, int> data_association;
  data_association.first = -1;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for(int i = 0; i < x_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - x_infinite_rooms[i].node->estimate()(0), 2) + pow(corr_pose(1) - x_infinite_rooms[i].node->estimate()(1), 2));

      if(dist < min_dist) {
        min_dist = dist;
        data_association.first = x_infinite_rooms[i].id;
        data_association.second = i;
        RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "dist x corr %f", dist);
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for(int i = 0; i < y_infinite_rooms.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - y_infinite_rooms[i].node->estimate()(0), 2) + pow(corr_pose(1) - y_infinite_rooms[i].node->estimate()(1), 2));

      if(dist < min_dist) {
        min_dist = dist;
        data_association.first = y_infinite_rooms[i].id;
        data_association.second = i;
        RCLCPP_DEBUG(node_obj->get_logger(), "infinite_room planes", "dist y corr %f", dist);
      }
    }
  }

  // RCLCPP_DEBUG(node_obj->get_logger(),"infinite_room planes", "min dist %f", min_dist);
  if(min_dist > infinite_room_dist_threshold) data_association.first = -1;

  return data_association;
}

double InfiniteRoomMapper::infinite_room_measurement(const int plane_type, const Eigen::Vector2d& infinite_room_pose, const Eigen::Vector4d& plane) {
  double meas;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(infinite_room_pose(0)) > fabs(plane(3))) {
      meas = infinite_room_pose(0) - plane(3);
    } else {
      meas = plane(3) - infinite_room_pose(0);
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(infinite_room_pose(1)) > fabs(plane(3))) {
      meas = infinite_room_pose(1) - plane(3);
    } else {
      meas = plane(3) - infinite_room_pose(1);
    }
  }

  return meas;
}

double InfiniteRoomMapper::infinite_room_measurement(const int plane_type, const Eigen::Vector2d& infinite_room_pose, const Eigen::Vector4d& plane1, const Eigen::Vector4d& plane2) {
  double meas;
  double plane_diff;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(plane1(3)) > fabs(plane2(3))) {
      double size = plane1(3) - plane2(3);
      plane_diff = ((size) / 2) + plane2(3);
    } else {
      double size = plane2(3) - plane1(3);
      plane_diff = ((size) / 2) + plane1(3);
    }
    meas = infinite_room_pose(0) - plane_diff;
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(plane1(3)) > fabs(plane2(3))) {
      double size = plane1(3) - plane2(3);
      plane_diff = ((size) / 2) + plane2(3);
    } else {
      double size = plane2(3) - plane1(3);
      plane_diff = ((size) / 2) + plane1(3);
    }
    meas = infinite_room_pose(1) - plane_diff;
  }

  return meas;
}

bool InfiniteRoomMapper::check_infinite_room_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexRoomXYLB* corr_node) {
  for(auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if(edge_infinite_room_planes) {
        g2o::VertexRoomXYLB* found_infinite_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_infinite_room_planes->vertices()[0]);
        if(found_infinite_room_node->id() == corr_node->id()) return true;
      }
    }

    if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_infinite_room_planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if(edge_infinite_room_planes) {
        g2o::VertexRoomXYLB* found_infinite_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_infinite_room_planes->vertices()[0]);
        if(found_infinite_room_node->id() == corr_node->id()) return true;
      }
    }
  }
  return false;
}

}  // namespace s_graphs
