// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_mapper.hpp>

namespace s_graphs {

InfiniteRoomMapper::InfiniteRoomMapper(const ros::NodeHandle& private_nh) {
  corridor_point_diff_threshold = private_nh.param<double>("corridor_point_diff_threshold", 3.0);
  corridor_plane_length_diff_threshold = private_nh.param<double>("corridor_plane_length_diff_threshold", 0.3);
  corridor_min_width = private_nh.param<double>("corridor_min_width", 1.5);
  corridor_max_width = private_nh.param<double>("corridor_max_width", 2.5);

  corridor_information = private_nh.param<double>("corridor_information", 0.01);
  corridor_dist_threshold = private_nh.param<double>("corridor_dist_threshold", 1.0);

  use_parallel_plane_constraint = private_nh.param<bool>("use_parallel_plane_constraint", true);
  use_perpendicular_plane_constraint = private_nh.param<bool>("use_perpendicular_plane_constraint", true);

  corridor_min_seg_dist = private_nh.param<double>("corridor_min_seg_dist", 1.5);

  plane_utils.reset(new PlaneUtils());
}

InfiniteRoomMapper::~InfiniteRoomMapper() {}

void InfiniteRoomMapper::lookup_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const std::vector<plane_data_list>& x_det_corridor_candidates, const std::vector<plane_data_list>& y_det_corridor_candidates, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Infinite_rooms>& x_corridors, std::vector<Infinite_rooms>& y_corridors) {
  std::vector<structure_data_list> x_corridor = sort_corridors(PlaneUtils::plane_class::X_VERT_PLANE, x_det_corridor_candidates);
  std::vector<structure_data_list> y_corridor = sort_corridors(PlaneUtils::plane_class::Y_VERT_PLANE, y_det_corridor_candidates);

  std::vector<plane_data_list> x_corridor_refined = refine_corridors(x_corridor);
  if(x_corridor_refined.size() == 2) factor_corridors(graph_slam, PlaneUtils::plane_class::X_VERT_PLANE, x_corridor_refined[0], x_corridor_refined[1], x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors);

  std::vector<plane_data_list> y_corridor_refined = refine_corridors(y_corridor);
  if(y_corridor_refined.size() == 2) factor_corridors(graph_slam, PlaneUtils::plane_class::Y_VERT_PLANE, y_corridor_refined[0], y_corridor_refined[1], x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors);
}

void InfiniteRoomMapper::lookup_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, const s_graphs::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Infinite_rooms>& x_corridors, std::vector<Infinite_rooms>& y_corridors, const std::vector<Rooms>& rooms_vec) {
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
      std::cout << "Room already exists in the given location, not inserting an x corridor" << std::endl;
      return;
    }

    // factor the corridor here
    std::cout << "factoring x corridor" << std::endl;
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

    // get the corridor neighbours
    for(const auto& room_neighbour_id : room_data.neighbour_ids) {
      x_plane1_data.connected_neighbour_ids.push_back(room_neighbour_id);
    }
    factor_corridors(graph_slam, PlaneUtils::plane_class::X_VERT_PLANE, x_plane1_data, x_plane2_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors);
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
      std::cout << "Room already exists in the given location, not inserting a y corridor" << std::endl;
      return;
    }

    // factor the corridor here
    std::cout << "factoring y corridor" << std::endl;
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

    // get the corridor neighbours
    for(const auto& room_neighbour_id : room_data.neighbour_ids) {
      y_plane1_data.connected_neighbour_ids.push_back(room_neighbour_id);
    }
    factor_corridors(graph_slam, PlaneUtils::plane_class::Y_VERT_PLANE, y_plane1_data, y_plane2_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors);
  }
}

std::vector<structure_data_list> InfiniteRoomMapper::sort_corridors(const int plane_type, const std::vector<plane_data_list>& corridor_candidates) {
  std::vector<structure_data_list> corridor_pair_vec;

  for(int i = 0; i < corridor_candidates.size(); ++i) {
    for(int j = i + 1; j < corridor_candidates.size(); ++j) {
      float corr_width = plane_utils->width_between_planes(corridor_candidates[i].plane_unflipped.coeffs(), corridor_candidates[j].plane_unflipped.coeffs());
      float diff_plane_length = fabs(corridor_candidates[i].plane_length - corridor_candidates[j].plane_length);
      float start_point_diff = MapperUtils::point_difference(plane_type, corridor_candidates[i].start_point, corridor_candidates[j].start_point);
      float end_point_diff = MapperUtils::point_difference(plane_type, corridor_candidates[i].end_point, corridor_candidates[j].end_point);
      float avg_plane_point_diff = (start_point_diff + end_point_diff) / 2;
      ROS_DEBUG_NAMED("corridor planes", "corr plane i coeffs %f %f %f %f", corridor_candidates[i].plane_unflipped.coeffs()(0), corridor_candidates[i].plane_unflipped.coeffs()(1), corridor_candidates[i].plane_unflipped.coeffs()(2),
                      corridor_candidates[i].plane_unflipped.coeffs()(3));
      ROS_DEBUG_NAMED("corridor planes", "corr plane j coeffs %f %f %f %f", corridor_candidates[j].plane_unflipped.coeffs()(0), corridor_candidates[j].plane_unflipped.coeffs()(1), corridor_candidates[j].plane_unflipped.coeffs()(2),
                      corridor_candidates[j].plane_unflipped.coeffs()(3));
      ROS_DEBUG_NAMED("corridor planes", "corr width %f", corr_width);
      ROS_DEBUG_NAMED("corridor planes", "plane length diff %f", diff_plane_length);
      ROS_DEBUG_NAMED("corridor planes", "avg plane point diff %f", avg_plane_point_diff);

      if(corridor_candidates[i].plane_unflipped.coeffs().head(3).dot(corridor_candidates[j].plane_unflipped.coeffs().head(3)) < 0 && (corr_width < corridor_max_width && corr_width > corridor_min_width) && diff_plane_length < corridor_plane_length_diff_threshold) {
        if(avg_plane_point_diff < corridor_point_diff_threshold) {
          structure_data_list corridor_pair;
          corridor_pair.plane1 = corridor_candidates[i];
          corridor_pair.plane2 = corridor_candidates[j];
          corridor_pair.width = corr_width;
          corridor_pair.length_diff = diff_plane_length;
          corridor_pair.avg_point_diff = avg_plane_point_diff;
          corridor_pair_vec.push_back(corridor_pair);
          ROS_DEBUG_NAMED("corridor planes", "adding corridor candidates");
        }
      }
    }
  }

  return corridor_pair_vec;
}

std::vector<plane_data_list> InfiniteRoomMapper::refine_corridors(const std::vector<structure_data_list>& corr_vec) {
  float min_corridor_diff = corridor_point_diff_threshold;
  std::vector<plane_data_list> corr_refined;
  corr_refined.resize(2);

  for(int i = 0; i < corr_vec.size(); ++i) {
    float corridor_diff = corr_vec[i].avg_point_diff;
    if(corridor_diff < min_corridor_diff) {
      min_corridor_diff = corridor_diff;
      corr_refined[0] = corr_vec[i].plane1;
      corr_refined[1] = corr_vec[i].plane2;
    }
  }

  if(min_corridor_diff >= corridor_point_diff_threshold) {
    std::vector<plane_data_list> corr_empty;
    corr_empty.resize(0);
    return corr_empty;
  } else
    return corr_refined;
}

void InfiniteRoomMapper::factor_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const int plane_type, const plane_data_list& corr_plane1_pair, const plane_data_list& corr_plane2_pair, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Infinite_rooms>& x_corridors, std::vector<Infinite_rooms>& y_corridors) {
  g2o::VertexRoomXYLB* corr_node;
  g2o::VertexRoomXYLB* cluster_center_node;
  std::pair<int, int> corr_data_association;
  double meas_plane1, meas_plane2;

  bool use_tri_edge = true;
  Eigen::Matrix<double, 2, 2> information_corridor_planes;
  information_corridor_planes(0, 0) = corridor_information;
  information_corridor_planes(1, 1) = corridor_information;

  Eigen::Matrix<double, 1, 1> information_corridor_plane;
  information_corridor_plane(0, 0) = corridor_information;

  Eigen::Matrix<double, 1, 1> information_corridor_prior;
  information_corridor_prior(0, 0) = 1e-5;

  // Eigen::Vector2d corr_pose = compute_corridor_pose(plane_type, corr_plane1_pair.plane_centroid, corr_plane1_pair.plane_unflipped.coeffs(), corr_plane2_pair.plane_unflipped.coeffs());
  Eigen::Vector2d corr_pose(corr_plane1_pair.plane_centroid(0), corr_plane1_pair.plane_centroid(1));
  ROS_DEBUG_NAMED("corridor planes", "final corridor plane 1 %f %f %f %f", corr_plane1_pair.plane_unflipped.coeffs()(0), corr_plane1_pair.plane_unflipped.coeffs()(1), corr_plane1_pair.plane_unflipped.coeffs()(2), corr_plane1_pair.plane_unflipped.coeffs()(3));
  ROS_DEBUG_NAMED("corridor planes", "final corridor plane 2 %f %f %f %f", corr_plane2_pair.plane_unflipped.coeffs()(0), corr_plane2_pair.plane_unflipped.coeffs()(1), corr_plane2_pair.plane_unflipped.coeffs()(2), corr_plane2_pair.plane_unflipped.coeffs()(3));

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    auto found_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
    auto found_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);

    if(found_plane1 == x_vert_planes.end() || found_plane2 == x_vert_planes.end()) {
      std::cout << "did not find planes for x corridor " << std::endl;
      return;
    }

    corr_data_association = associate_corridors(plane_type, corr_pose, (*found_plane1), (*found_plane2), x_vert_planes, y_vert_planes, x_corridors, y_corridors);

    if((x_corridors.empty() || corr_data_association.first == -1)) {
      std::cout << "found an X corridor with pre pose " << corr_pose << " between plane " << corr_plane1_pair.plane_unflipped.coeffs() << " and plane " << corr_plane2_pair.plane_unflipped.coeffs() << std::endl;

      corr_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      corr_node = graph_slam->add_room_node(corr_pose);
      cluster_center_node = graph_slam->add_room_node(corr_plane1_pair.cluster_center);
      cluster_center_node->setFixed(true);
      // graph_slam->add_room_yprior_edge(corr_node, corr_pose(1), information_corridor_prior);
      Infinite_rooms det_corridor;
      det_corridor.id = corr_data_association.first;
      det_corridor.plane1 = corr_plane1_pair.plane_unflipped;
      det_corridor.plane2 = corr_plane2_pair.plane_unflipped;
      det_corridor.plane1_id = corr_plane1_pair.plane_id;
      det_corridor.plane2_id = corr_plane2_pair.plane_id;
      det_corridor.cluster_center_node = cluster_center_node;
      det_corridor.node = corr_node;
      det_corridor.connected_id = corr_plane1_pair.connected_id;
      det_corridor.connected_neighbour_ids = corr_plane1_pair.connected_neighbour_ids;
      x_corridors.push_back(det_corridor);

      if(use_tri_edge) {
        auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, cluster_center_node, information_corridor_planes);
        graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);
      } else {
        meas_plane1 = corridor_measurement(plane_type, corr_pose, corr_plane1_pair.plane_unflipped.coeffs());
        meas_plane2 = corridor_measurement(plane_type, corr_pose, corr_plane2_pair.plane_unflipped.coeffs());
        /* Add parallel constraints here */
        if(use_parallel_plane_constraint) {
          MapperUtils::parallel_plane_constraint(graph_slam, (*found_plane1).plane_node, (*found_plane2).plane_node);
        }
        auto edge_plane1 = graph_slam->add_room_xplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
        graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);
        auto edge_plane2 = graph_slam->add_room_xplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
        graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
      }

    } else {
      /* add the edge between detected planes and the corridor */
      corr_node = x_corridors[corr_data_association.second].node;
      std::cout << "Matched det corridor X with pre pose " << corr_pose << " to mapped corridor with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

      x_corridors[corr_data_association.second].connected_id = corr_plane1_pair.connected_id;
      for(const auto& det_corr_neighbour : corr_plane1_pair.connected_neighbour_ids) {
        x_corridors[corr_data_association.second].connected_neighbour_ids.push_back(det_corr_neighbour);
      }

      auto found_mapped_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[corr_data_association.second].plane1_id);
      auto found_mapped_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[corr_data_association.second].plane2_id);

      Eigen::Vector4d found_mapped_plane1_coeffs, found_mapped_plane2_coeffs;
      found_mapped_plane1_coeffs = (*found_mapped_plane1).plane_node->estimate().coeffs();
      found_mapped_plane2_coeffs = (*found_mapped_plane2).plane_node->estimate().coeffs();
      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_plane1_coeffs);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_plane2_coeffs);

      if(use_tri_edge) {
        std::set<g2o::HyperGraph::Edge*> plane1_edges = (*found_plane1).plane_node->edges();
        std::set<g2o::HyperGraph::Edge*> plane2_edges = (*found_plane2).plane_node->edges();

        if(!check_corridor_ids(plane_type, plane1_edges, corr_node) || !check_corridor_ids(plane_type, plane2_edges, corr_node)) {
          std::cout << "adding new x1 plane and x2 plane edges with corridor " << std::endl;
          auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, x_corridors[corr_data_association.second].cluster_center_node, information_corridor_planes);
          graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);
        }
      } else {
        bool found_new_plane = false;
        if((*found_plane1).id == (*found_mapped_plane1).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane1).id == (*found_mapped_plane2).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane1).plane_node->estimate().coeffs().head(3)).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane1);
          } else {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_x_vert_planes.push_back(dupl_plane_pair);

          std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_plane1).plane_node->edges();
          if(!check_corridor_ids(plane_type, plane_edges, corr_node)) {
            std::cout << "adding x1 plane edge with corridor " << std::endl;
            auto edge_plane1 = graph_slam->add_room_xplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
            graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);
          }
        }

        if((*found_plane2).id == (*found_mapped_plane1).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane2).id == (*found_mapped_plane2).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane2).plane_node->estimate().coeffs().head(3)).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane1);
          } else {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_x_vert_planes.push_back(dupl_plane_pair);

          std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_plane2).plane_node->edges();
          if(!check_corridor_ids(plane_type, plane_edges, corr_node)) {
            std::cout << "adding x2 plane edge corridor " << std::endl;
            auto edge_plane2 = graph_slam->add_room_xplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
            graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
          }
        }

        // std::cout << "x mapped plane1 id : " << (*found_mapped_plane1).id << std::endl;
        // std::cout << "x mapped plane1 coeffs : " << (*found_mapped_plane1).plane_node->estimate().coeffs() << std::endl;
        // std::cout << "x mapped plane2 id : " << (*found_mapped_plane2).id << std::endl;
        // std::cout << "x mapped plane2 coeffs : " << (*found_mapped_plane2).plane_node->estimate().coeffs() << std::endl;

        // std::cout << "x found plane1 id : " << (*found_plane1).id << std::endl;
        // std::cout << "x found plane1 coeffs : " << (*found_plane1).plane_node->estimate().coeffs() << std::endl;
        // std::cout << "x found plane2 id : " << (*found_plane2).id << std::endl;
        // std::cout << "x found plane2 coeffs : " << (*found_plane2).plane_node->estimate().coeffs() << std::endl;

        if(use_parallel_plane_constraint && found_new_plane) {
          MapperUtils::parallel_plane_constraint(graph_slam, (*found_plane1).plane_node, (*found_plane2).plane_node);
        }
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    auto found_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
    auto found_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);

    if(found_plane1 == y_vert_planes.end() || found_plane2 == y_vert_planes.end()) return;

    corr_data_association = associate_corridors(plane_type, corr_pose, (*found_plane1), (*found_plane2), x_vert_planes, y_vert_planes, x_corridors, y_corridors);

    if((y_corridors.empty() || corr_data_association.first == -1)) {
      std::cout << "found an Y corridor with pre pose " << corr_pose << " between plane " << corr_plane1_pair.plane_unflipped.coeffs() << " and plane " << corr_plane2_pair.plane_unflipped.coeffs() << std::endl;

      corr_data_association.first = graph_slam->retrieve_local_nbr_of_vertices();
      corr_node = graph_slam->add_room_node(corr_pose);
      cluster_center_node = graph_slam->add_room_node(corr_plane1_pair.cluster_center);
      cluster_center_node->setFixed(true);
      // graph_slam->add_room_xprior_edge(corr_node, corr_pose(0), information_corridor_prior);

      Infinite_rooms det_corridor;
      det_corridor.id = corr_data_association.first;
      det_corridor.plane1 = corr_plane1_pair.plane_unflipped;
      det_corridor.plane2 = corr_plane2_pair.plane_unflipped;
      det_corridor.plane1_id = corr_plane1_pair.plane_id;
      det_corridor.plane2_id = corr_plane2_pair.plane_id;
      det_corridor.cluster_center_node = cluster_center_node;
      det_corridor.node = corr_node;
      det_corridor.connected_id = corr_plane1_pair.connected_id;
      det_corridor.connected_neighbour_ids = corr_plane1_pair.connected_neighbour_ids;
      y_corridors.push_back(det_corridor);

      if(use_tri_edge) {
        auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, cluster_center_node, information_corridor_planes);
        graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);
      } else {
        meas_plane1 = corridor_measurement(plane_type, corr_pose, corr_plane1_pair.plane_unflipped.coeffs());
        meas_plane2 = corridor_measurement(plane_type, corr_pose, corr_plane2_pair.plane_unflipped.coeffs());

        if(use_parallel_plane_constraint) {
          MapperUtils::parallel_plane_constraint(graph_slam, (*found_plane1).plane_node, (*found_plane2).plane_node);
        }

        auto edge_plane1 = graph_slam->add_room_yplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
        graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);

        auto edge_plane2 = graph_slam->add_room_yplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
        graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
      }
    } else {
      /* add the edge between detected planes and the corridor */
      corr_node = y_corridors[corr_data_association.second].node;
      std::cout << "Matched det corridor Y with pre pose " << corr_pose << " to mapped corridor with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

      y_corridors[corr_data_association.second].connected_id = corr_plane1_pair.connected_id;
      for(const auto& det_corr_neighbour : corr_plane1_pair.connected_neighbour_ids) {
        y_corridors[corr_data_association.second].connected_neighbour_ids.push_back(det_corr_neighbour);
      }
      auto found_mapped_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[corr_data_association.second].plane1_id);
      auto found_mapped_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[corr_data_association.second].plane2_id);

      Eigen::Vector4d found_mapped_plane1_coeffs, found_mapped_plane2_coeffs;
      found_mapped_plane1_coeffs = (*found_mapped_plane1).plane_node->estimate().coeffs();
      found_mapped_plane2_coeffs = (*found_mapped_plane2).plane_node->estimate().coeffs();
      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_plane1_coeffs);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_plane2_coeffs);

      if(use_tri_edge) {
        std::set<g2o::HyperGraph::Edge*> plane1_edges = (*found_plane1).plane_node->edges();
        std::set<g2o::HyperGraph::Edge*> plane2_edges = (*found_plane2).plane_node->edges();

        if(!check_corridor_ids(plane_type, plane1_edges, corr_node) || !check_corridor_ids(plane_type, plane2_edges, corr_node)) {
          std::cout << "adding new y1 plane and y2 plane edges with corridor " << std::endl;
          auto edge_corr_plane = graph_slam->add_room_2planes_edge(corr_node, (*found_plane1).plane_node, (*found_plane2).plane_node, y_corridors[corr_data_association.second].cluster_center_node, information_corridor_planes);
          graph_slam->add_robust_kernel(edge_corr_plane, "Huber", 1.0);
        }
      } else {
        bool found_new_plane = false;
        if((*found_plane1).id == (*found_mapped_plane1).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane1).id == (*found_mapped_plane2).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane1).plane_node->estimate().coeffs().head(3)).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane1);
          } else {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_y_vert_planes.push_back(dupl_plane_pair);

          std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_plane1).plane_node->edges();
          if(!check_corridor_ids(plane_type, plane_edges, corr_node)) {
            std::cout << "adding y1 plane edge with corridor " << std::endl;
            auto edge_plane1 = graph_slam->add_room_yplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
            graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);
          }
        }

        if((*found_plane2).id == (*found_mapped_plane1).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane2).id == (*found_mapped_plane2).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane2).plane_node->estimate().coeffs().head(3)).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane1);
          } else {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_y_vert_planes.push_back(dupl_plane_pair);

          std::set<g2o::HyperGraph::Edge*> plane_edges = (*found_plane2).plane_node->edges();
          if(!check_corridor_ids(plane_type, plane_edges, corr_node)) {
            std::cout << "adding y2 plane edge corridor " << std::endl;
            auto edge_plane2 = graph_slam->add_room_yplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
            graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
          }
        }

        // std::cout << "y mapped plane1 id : " << (*found_mapped_plane1).id << std::endl;
        // std::cout << "y mapped plane1 coeffs : " << (*found_mapped_plane1).plane_node->estimate().coeffs() << std::endl;
        // std::cout << "y mapped plane2 id : " << (*found_mapped_plane2).id << std::endl;
        // std::cout << "y mapped plane2 coeffs : " << (*found_mapped_plane2).plane_node->estimate().coeffs() << std::endl;

        // std::cout << "y found plane1 id : " << (*found_plane1).id << std::endl;
        // std::cout << "y found plane1 coeffs : " << (*found_plane1).plane_node->estimate().coeffs() << std::endl;
        // std::cout << "y found plane2 id : " << (*found_plane2).id << std::endl;
        // std::cout << "y found plane2 coeffs : " << (*found_plane2).plane_node->estimate().coeffs() << std::endl;

        if(use_parallel_plane_constraint && found_new_plane) {
          MapperUtils::parallel_plane_constraint(graph_slam, (*found_plane1).plane_node, (*found_plane2).plane_node);
        }
      }
    }
  }

  return;
}

std::pair<int, int> InfiniteRoomMapper::associate_corridors(const int& plane_type, const Eigen::Vector2d& corr_pose, const VerticalPlanes& plane1, const VerticalPlanes& plane2, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const std::vector<Infinite_rooms>& x_corridors, const std::vector<Infinite_rooms>& y_corridors) {
  float min_dist = 100;
  bool plane1_min_segment = false, plane2_min_segment = false;

  std::pair<int, int> data_association;
  data_association.first = -1;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for(int i = 0; i < x_corridors.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - x_corridors[i].node->estimate()(0), 2));

      auto found_mapped_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[i].plane2_id);

      if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
      } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
      } else
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);

      if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
      } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
      } else
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);

      if(dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = x_corridors[i].id;
        data_association.second = i;
        ROS_DEBUG_NAMED("corridor planes", "dist x corr %f", dist);
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for(int i = 0; i < y_corridors.size(); ++i) {
      float dist = sqrt(pow(corr_pose(1) - y_corridors[i].node->estimate()(1), 2));

      auto found_mapped_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[i].plane1_id);
      auto found_mapped_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[i].plane2_id);

      if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
        plane1_min_segment = true;
      } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
      } else
        plane1_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);

      if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
        plane2_min_segment = true;
      } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
      } else
        plane2_min_segment = plane_utils->check_point_neighbours((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);

      if(dist < min_dist && (plane1_min_segment && plane2_min_segment)) {
        min_dist = dist;
        data_association.first = y_corridors[i].id;
        data_association.second = i;
        ROS_DEBUG_NAMED("corridor planes", "dist y corr %f", dist);
      }
    }
  }

  // ROS_DEBUG_NAMED("corridor planes", "min dist %f", min_dist);
  if(min_dist > corridor_dist_threshold) data_association.first = -1;

  return data_association;
}

std::pair<int, int> InfiniteRoomMapper::associate_corridors(const int& plane_type, const Eigen::Vector2d& corr_pose, const std::vector<Infinite_rooms>& x_corridors, const std::vector<Infinite_rooms>& y_corridors) {
  float min_dist = 100;
  std::pair<int, int> data_association;
  data_association.first = -1;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    for(int i = 0; i < x_corridors.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - x_corridors[i].node->estimate()(0), 2) + pow(corr_pose(1) - x_corridors[i].node->estimate()(1), 2));

      if(dist < min_dist) {
        min_dist = dist;
        data_association.first = x_corridors[i].id;
        data_association.second = i;
        ROS_DEBUG_NAMED("corridor planes", "dist x corr %f", dist);
      }
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    for(int i = 0; i < y_corridors.size(); ++i) {
      float dist = sqrt(pow(corr_pose(0) - y_corridors[i].node->estimate()(0), 2) + pow(corr_pose(1) - y_corridors[i].node->estimate()(1), 2));

      if(dist < min_dist) {
        min_dist = dist;
        data_association.first = y_corridors[i].id;
        data_association.second = i;
        ROS_DEBUG_NAMED("corridor planes", "dist y corr %f", dist);
      }
    }
  }

  // ROS_DEBUG_NAMED("corridor planes", "min dist %f", min_dist);
  if(min_dist > corridor_dist_threshold) data_association.first = -1;

  return data_association;
}

double InfiniteRoomMapper::corridor_measurement(const int plane_type, const Eigen::Vector2d& corridor_pose, const Eigen::Vector4d& plane) {
  double meas;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(corridor_pose(0)) > fabs(plane(3))) {
      meas = corridor_pose(0) - plane(3);
    } else {
      meas = plane(3) - corridor_pose(0);
    }
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(corridor_pose(1)) > fabs(plane(3))) {
      meas = corridor_pose(1) - plane(3);
    } else {
      meas = plane(3) - corridor_pose(1);
    }
  }

  return meas;
}

double InfiniteRoomMapper::corridor_measurement(const int plane_type, const Eigen::Vector2d& corridor_pose, const Eigen::Vector4d& plane1, const Eigen::Vector4d& plane2) {
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
    meas = corridor_pose(0) - plane_diff;
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(plane1(3)) > fabs(plane2(3))) {
      double size = plane1(3) - plane2(3);
      plane_diff = ((size) / 2) + plane2(3);
    } else {
      double size = plane2(3) - plane1(3);
      plane_diff = ((size) / 2) + plane1(3);
    }
    meas = corridor_pose(1) - plane_diff;
  }

  return meas;
}

bool InfiniteRoomMapper::check_corridor_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexRoomXYLB* corr_node) {
  for(auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end(); ++edge_itr) {
    if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_corridor_planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if(edge_corridor_planes) {
        g2o::VertexRoomXYLB* found_corridor_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_corridor_planes->vertices()[0]);
        if(found_corridor_node->id() == corr_node->id()) return true;
      }
    }

    if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      g2o::EdgeRoom2Planes* edge_corridor_planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      if(edge_corridor_planes) {
        g2o::VertexRoomXYLB* found_corridor_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_corridor_planes->vertices()[0]);
        if(found_corridor_node->id() == corr_node->id()) return true;
      }
    }
  }
  return false;
}

}  // namespace s_graphs
