#include <s_graphs/backend/plane_mapper.hpp>

namespace s_graphs {

PlaneMapper::PlaneMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
  node_obj = node;
  use_point_to_plane =
      node_obj->get_parameter("use_point_to_plane").get_parameter_value().get<bool>();
  plane_information =
      node_obj->get_parameter("plane_information").get_parameter_value().get<double>();
  dupl_plane_matching_information =
      node->get_parameter("dupl_plane_matching_information")
          .get_parameter_value()
          .get<double>();

  plane_dist_threshold = node_obj->get_parameter("plane_dist_threshold")
                             .get_parameter_value()
                             .get<double>();

  plane_points_dist =
      node_obj->get_parameter("plane_points_dist").get_parameter_value().get<double>();
  min_plane_points =
      node_obj->get_parameter("min_plane_points").get_parameter_value().get<int>();
  min_plane_points_opti =
      node_obj->get_parameter("min_plane_points").get_parameter_value().get<int>();
}

PlaneMapper::~PlaneMapper() {}

VerticalPlanes PlaneMapper::add_plane(std::shared_ptr<GraphSLAM>& covisibility_graph,
                                      g2o::VertexPlane* plane) {
  int id = covisibility_graph->retrieve_local_nbr_of_vertices();
  auto plane_node = covisibility_graph->add_plane_node(plane->estimate().coeffs());
  VerticalPlanes vert_plane;
  vert_plane.id = id;
  vert_plane.plane_node = plane_node;
  vert_plane.cloud_seg_map =
      pcl::PointCloud<PointNormal>::Ptr(new pcl::PointCloud<PointNormal>);
  vert_plane.covariance = Eigen::Matrix3d::Identity();
  std::vector<double> color;
  color.push_back(rand() % 256);  // red
  color.push_back(rand() % 256);  // green
  color.push_back(rand() % 256);  // blue
  color.push_back(255);           // alpha

  vert_plane.color = color;

  return vert_plane;
}

void PlaneMapper::factor_new_planes(
    const int& current_floor_level,
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::vector<g2o::VertexPlane*> new_x_planes,
    const std::vector<g2o::VertexPlane*> new_y_planes,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes) {
  for (const auto& new_x_plane : new_x_planes) {
    VerticalPlanes vert_plane = this->add_plane(covisibility_graph, new_x_plane);

    vert_plane.floor_level = current_floor_level;
    shared_graph_mutex.lock();
    x_vert_planes.insert({vert_plane.id, vert_plane});
    shared_graph_mutex.unlock();
  }

  for (const auto& new_y_plane : new_y_planes) {
    VerticalPlanes vert_plane = this->add_plane(covisibility_graph, new_y_plane);

    vert_plane.floor_level = current_floor_level;
    shared_graph_mutex.lock();
    y_vert_planes.insert({vert_plane.id, vert_plane});
    shared_graph_mutex.unlock();
  }
}

void PlaneMapper::map_extracted_planes(
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    KeyFrame::Ptr keyframe,
    const std::vector<pcl::PointCloud<PointNormal>::Ptr>& extracted_cloud_vec,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes) {
  std::vector<plane_data_list> x_det_infinite_room_candidates,
      y_det_infinite_room_candidates;
  std::vector<plane_data_list> x_det_room_candidates, y_det_room_candidates;

  for (const auto& cloud_seg_body : extracted_cloud_vec) {
    if (cloud_seg_body->points.size() < min_plane_points) continue;

    g2o::Plane3D det_plane_body_frame =
        Eigen::Vector4d(cloud_seg_body->back().normal_x,
                        cloud_seg_body->back().normal_y,
                        cloud_seg_body->back().normal_z,
                        cloud_seg_body->back().curvature);
    int plane_type = add_planes_to_graph(covisibility_graph,
                                         keyframe,
                                         cloud_seg_body,
                                         det_plane_body_frame,
                                         x_vert_planes,
                                         y_vert_planes,
                                         hort_planes);
  }

  convert_plane_points_to_map(x_vert_planes, y_vert_planes, hort_planes, keyframe);
}

/**
 * @brief detected plane mapping
 *
 */
int PlaneMapper::add_planes_to_graph(
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    const KeyFrame::Ptr& keyframe,
    const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
    const g2o::Plane3D& det_plane_body_frame,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes) {
  int plane_id;
  int plane_type = -1;

  g2o::Plane3D det_plane_map_frame =
      convert_plane_to_map_frame(keyframe, det_plane_body_frame);

  /* Get the plane type based on the largest value of the normal orientation x,y and z
   */
  if (fabs(det_plane_map_frame.coeffs()(0)) > fabs(det_plane_map_frame.coeffs()(1)) &&
      fabs(det_plane_map_frame.coeffs()(0)) > fabs(det_plane_map_frame.coeffs()(2)))
    plane_type = PlaneUtils::plane_class::X_VERT_PLANE;
  else if (fabs(det_plane_map_frame.coeffs()(1)) >
               fabs(det_plane_map_frame.coeffs()(0)) &&
           fabs(det_plane_map_frame.coeffs()(1)) >
               fabs(det_plane_map_frame.coeffs()(2)))
    plane_type = PlaneUtils::plane_class::Y_VERT_PLANE;
  else if (fabs(det_plane_map_frame.coeffs()(2)) >
               fabs(det_plane_map_frame.coeffs()(0)) &&
           fabs(det_plane_map_frame.coeffs()(2)) >
               fabs(det_plane_map_frame.coeffs()(1)))
    plane_type = PlaneUtils::plane_class::HORT_PLANE;

  plane_id = sort_planes(covisibility_graph,
                         plane_type,
                         keyframe,
                         cloud_seg_body,
                         det_plane_map_frame,
                         det_plane_body_frame,
                         x_vert_planes,
                         y_vert_planes,
                         hort_planes);

  return plane_type;
}

/**
 * @brief convert body plane coefficients to map frame
 */
g2o::Plane3D PlaneMapper::convert_plane_to_map_frame(
    const KeyFrame::Ptr& keyframe,
    const g2o::Plane3D& det_plane_body_frame) {
  g2o::Plane3D det_plane_map_frame;
  Eigen::Vector4d map_coeffs;

  Eigen::Isometry3d w2n = keyframe->node->estimate();
  map_coeffs.head<3>() = w2n.rotation() * det_plane_body_frame.coeffs().head<3>();
  map_coeffs(3) =
      det_plane_body_frame.coeffs()(3) - w2n.translation().dot(map_coeffs.head<3>());
  det_plane_map_frame = map_coeffs;

  return det_plane_map_frame;
}

/**
 * @brief sort and factor the detected planes
 *
 */
int PlaneMapper::sort_planes(std::shared_ptr<GraphSLAM>& covisibility_graph,
                             const int& plane_type,
                             const KeyFrame::Ptr& keyframe,
                             const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
                             const g2o::Plane3D& det_plane_map_frame,
                             const g2o::Plane3D& det_plane_body_frame,
                             std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                             std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                             std::unordered_map<int, HorizontalPlanes>& hort_planes) {
  int plane_id = factor_planes(covisibility_graph,
                               plane_type,
                               keyframe,
                               cloud_seg_body,
                               det_plane_map_frame,
                               det_plane_body_frame,
                               x_vert_planes,
                               y_vert_planes,
                               hort_planes);

  return plane_id;
}

/**
 * @brief create vertical plane factors
 */
int PlaneMapper::factor_planes(std::shared_ptr<GraphSLAM>& covisibility_graph,
                               const int& plane_type,
                               const KeyFrame::Ptr& keyframe,
                               const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
                               const g2o::Plane3D& det_plane_map_frame,
                               const g2o::Plane3D& det_plane_body_frame,
                               std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                               std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                               std::unordered_map<int, HorizontalPlanes>& hort_planes) {
  g2o::VertexPlane* plane_node;
  int plane_id = -1;

  Eigen::Matrix4d Gij;
  Gij.setZero();
  if (use_point_to_plane) {
    auto it = cloud_seg_body->points.begin();
    while (it != cloud_seg_body->points.end()) {
      PointNormal point_tmp;
      point_tmp = *it;
      Eigen::Vector4d point(point_tmp.x, point_tmp.y, point_tmp.z, 1);
      double point_to_plane_d = det_plane_map_frame.coeffs().transpose() *
                                keyframe->node->estimate().matrix() * point;

      if (abs(point_to_plane_d) < 0.1) {
        Gij += point * point.transpose();
        ++it;
      } else {
        it = cloud_seg_body->points.erase(it);
      }
    }
  }

  int data_association = -1;
  data_association = associate_plane(plane_type,
                                     keyframe,
                                     det_plane_body_frame.coeffs(),
                                     cloud_seg_body,
                                     x_vert_planes,
                                     y_vert_planes,
                                     hort_planes);
  int plane_points = cloud_seg_body->points.size();

  switch (plane_type) {
    case PlaneUtils::plane_class::X_VERT_PLANE: {
      if (x_vert_planes.empty() || data_association == -1) {
        VerticalPlanes vert_plane = add_new_plane<VerticalPlanes>(covisibility_graph,
                                                                  det_plane_body_frame,
                                                                  det_plane_map_frame,
                                                                  keyframe,
                                                                  cloud_seg_body,
                                                                  x_vert_planes);

        plane_node = vert_plane.plane_node;
        shared_graph_mutex.lock();
        keyframe->x_plane_ids.push_back(vert_plane.id);
        shared_graph_mutex.unlock();
      } else {
        plane_node = update_plane<VerticalPlanes>(
            data_association, keyframe, cloud_seg_body, x_vert_planes);
        shared_graph_mutex.lock();
        keyframe->x_plane_ids.push_back(x_vert_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
    case PlaneUtils::plane_class::Y_VERT_PLANE: {
      if (y_vert_planes.empty() || data_association == -1) {
        VerticalPlanes vert_plane = add_new_plane<VerticalPlanes>(covisibility_graph,
                                                                  det_plane_body_frame,
                                                                  det_plane_map_frame,
                                                                  keyframe,
                                                                  cloud_seg_body,
                                                                  y_vert_planes);

        plane_node = vert_plane.plane_node;
        shared_graph_mutex.lock();
        keyframe->y_plane_ids.push_back(vert_plane.id);
        shared_graph_mutex.unlock();
      } else {
        plane_node = update_plane<VerticalPlanes>(
            data_association, keyframe, cloud_seg_body, y_vert_planes);
        shared_graph_mutex.lock();
        keyframe->y_plane_ids.push_back(y_vert_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
    case PlaneUtils::plane_class::HORT_PLANE: {
      if (hort_planes.empty() || data_association == -1) {
        HorizontalPlanes hort_plane =
            add_new_plane<HorizontalPlanes>(covisibility_graph,
                                            det_plane_body_frame,
                                            det_plane_map_frame,
                                            keyframe,
                                            cloud_seg_body,
                                            hort_planes);

        plane_node = hort_plane.plane_node;
        shared_graph_mutex.lock();
        keyframe->hort_plane_ids.push_back(hort_plane.id);
        shared_graph_mutex.unlock();
      } else {
        plane_node = update_plane<HorizontalPlanes>(
            data_association, keyframe, cloud_seg_body, hort_planes);
        shared_graph_mutex.lock();
        keyframe->hort_plane_ids.push_back(hort_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
  }

  shared_graph_mutex.lock();
  if (use_point_to_plane) {
    Eigen::Matrix<double, 1, 1> information(0.001);
    auto edge = covisibility_graph->add_se3_point_to_plane_edge(
        keyframe->node, plane_node, Gij, information);
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
  } else {
    Eigen::Matrix3d plane_information_mat =
        Eigen::Matrix3d::Identity() * plane_information;

    // further reduce the info mat if too less points
    if (plane_points > min_plane_points_opti) {
      auto edge = covisibility_graph->add_se3_plane_edge(keyframe->node,
                                                         plane_node,
                                                         det_plane_body_frame.coeffs(),
                                                         plane_information_mat);
      covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    }
  }

  shared_graph_mutex.unlock();

  return data_association;
}

/**
 * @brief data assoction betweeen the planes
 */
int PlaneMapper::associate_plane(
    const int& plane_type,
    const KeyFrame::Ptr& keyframe,
    const g2o::Plane3D& det_plane,
    const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::unordered_map<int, HorizontalPlanes>& hort_planes) {
  int data_association = -1;

  switch (plane_type) {
    case PlaneUtils::plane_class::X_VERT_PLANE: {
      data_association = get_matched_planes<VerticalPlanes>(
          det_plane, keyframe, cloud_seg_body, x_vert_planes);
      break;
    }
    case PlaneUtils::plane_class::Y_VERT_PLANE: {
      data_association = get_matched_planes<VerticalPlanes>(
          det_plane, keyframe, cloud_seg_body, y_vert_planes);
      break;
    }
    case PlaneUtils::plane_class::HORT_PLANE: {
      data_association = get_matched_planes<HorizontalPlanes>(
          det_plane, keyframe, cloud_seg_body, hort_planes);
      break;
    }
    default:
      std::cout << "associating planes had an error " << std::endl;
      break;
  }

  return data_association;
}

template <typename T>
T PlaneMapper::add_new_plane(std::shared_ptr<GraphSLAM>& covisibility_graph,
                             const g2o::Plane3D& det_plane_body_frame,
                             const g2o::Plane3D& det_plane_map_frame,
                             const KeyFrame::Ptr& keyframe,
                             const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
                             std::unordered_map<int, T>& planes) {
  int id = covisibility_graph->retrieve_local_nbr_of_vertices();
  auto plane_node = covisibility_graph->add_plane_node(det_plane_map_frame.coeffs());

  T plane;
  plane.id = id;
  plane.cloud_seg_body_vec.push_back(cloud_seg_body);
  plane.keyframe_node_vec.push_back(keyframe->node);
  plane.plane_node = plane_node;
  plane.cloud_seg_map =
      pcl::PointCloud<PointNormal>::Ptr(new pcl::PointCloud<PointNormal>);
  plane.covariance = Eigen::Matrix3d::Identity();
  plane.floor_level = keyframe->floor_level;
  std::vector<double> color;
  color.push_back(cloud_seg_body->points.back().r);
  color.push_back(cloud_seg_body->points.back().g);
  color.push_back(cloud_seg_body->points.back().b);
  color.push_back(255);
  plane.color = color;

  shared_graph_mutex.lock();
  planes.insert({plane.id, plane});
  shared_graph_mutex.unlock();

  return plane;
}

template <typename T>
g2o::VertexPlane* PlaneMapper::update_plane(
    const int match_id,
    const KeyFrame::Ptr& keyframe,
    const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
    std::unordered_map<int, T>& planes) {
  shared_graph_mutex.lock();
  planes[match_id].cloud_seg_body_vec.push_back(cloud_seg_body);
  planes[match_id].keyframe_node_vec.push_back(keyframe->node);
  shared_graph_mutex.unlock();

  return planes[match_id].plane_node;
}

template <typename T>
int PlaneMapper::get_matched_planes(
    const g2o::Plane3D& det_plane,
    const KeyFrame::Ptr& keyframe,
    const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
    const std::unordered_map<int, T>& planes) {
  int data_association = -1;
  int min_maha_dist = 100;
  std::vector<std::pair<int, double>> potential_matches;

  shared_graph_mutex.lock();
  Eigen::Isometry3d n2m = keyframe->estimate();
  shared_graph_mutex.unlock();

  Eigen::Isometry3d m2n = n2m.inverse();
  Eigen::Matrix4f current_keyframe_pose = n2m.matrix().cast<float>();
  int current_floor_level = keyframe->floor_level;

  for (const auto& plane : planes) {
    if (current_floor_level != plane.second.floor_level) continue;

    shared_graph_mutex.lock();
    g2o::Plane3D local_plane = m2n * plane.second.plane_node->estimate();
    shared_graph_mutex.unlock();

    Eigen::Vector3d error = local_plane.ominus(det_plane);
    double maha_dist =
        sqrt(error.transpose() * plane.second.covariance.inverse() * error);

    if (std::isnan(maha_dist) || maha_dist < 1e-3) {
      Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
      maha_dist = sqrt(error.transpose() * cov * error);
    }
    if (maha_dist < plane_dist_threshold) {
      potential_matches.emplace_back(plane.second.id, maha_dist);
    }
  }

  if (!potential_matches.empty()) {
    std::sort(potential_matches.begin(),
              potential_matches.end(),
              [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second < b.second;
              });

    pcl::PointCloud<PointNormal>::Ptr cloud_seg_detected(
        new pcl::PointCloud<PointNormal>());
    for (size_t j = 0; j < cloud_seg_body->points.size(); ++j) {
      PointNormal dst_pt;
      dst_pt.getVector4fMap() =
          current_keyframe_pose * cloud_seg_body->points[j].getVector4fMap();
      cloud_seg_detected->points.push_back(dst_pt);
    }

    // Check all potential matches for point neighbors
    for (const auto& match : potential_matches) {
      int match_id = match.first;
      bool valid_neighbour = false;
      if (!planes.at(match_id).cloud_seg_map->points.empty()) {
        valid_neighbour = PlaneUtils::check_point_neighbours(
            planes.at(match_id).cloud_seg_map, cloud_seg_detected, plane_points_dist);
      }

      if (valid_neighbour && match.second < min_maha_dist) {
        min_maha_dist = match.second;
        data_association = match_id;
      }
    }

    // This will associate the closest plane which doesnt have any points but are in the
    // map
    if (data_association == -1) {
      if (planes.at(potential_matches.begin()->first).cloud_seg_map->points.empty()) {
        data_association = potential_matches.begin()->first;
      }
    }
  }

  return data_association;
}

void PlaneMapper::convert_plane_points_to_map(
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes,
    const KeyFrame::Ptr keyframe) {
  int current_floor_level = keyframe->floor_level;

  std::vector<int> x_plane_ids = keyframe->x_plane_ids;
  for (auto& x_plane_id : x_plane_ids) {
    auto x_vert_plane = x_vert_planes.find(x_plane_id);
    if (x_vert_plane->second.floor_level != current_floor_level) break;

    this->fill_plane_points<VerticalPlanes>(x_vert_plane->second);
  }

  std::vector<int> y_plane_ids = keyframe->y_plane_ids;
  for (auto& y_plane_id : y_plane_ids) {
    auto y_vert_plane = y_vert_planes.find(y_plane_id);
    if (y_vert_plane->second.floor_level != current_floor_level) break;

    this->fill_plane_points<VerticalPlanes>(y_vert_plane->second);
  }

  std::vector<int> hort_plane_ids = keyframe->hort_plane_ids;
  for (auto& hort_plane_id : hort_plane_ids) {
    auto hort_plane = hort_planes.find(hort_plane_id);

    if (hort_plane->second.floor_level != current_floor_level) break;

    this->fill_plane_points<HorizontalPlanes>(hort_plane->second);
  }
}

void PlaneMapper::convert_plane_points_to_map(
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes,
    std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>
        updated_planes_tuple) {
  for (auto& x_plane_id : std::get<0>(updated_planes_tuple)) {
    auto x_vert_plane = x_vert_planes.find(x_plane_id);
    this->fill_plane_points<VerticalPlanes>(x_vert_plane->second);
  }

  for (const auto& y_plane_id : std::get<1>(updated_planes_tuple)) {
    auto y_vert_plane = x_vert_planes.find(y_plane_id);
    this->fill_plane_points<VerticalPlanes>(y_vert_plane->second);
  }

  for (const auto& hort_plane_id : std::get<2>(updated_planes_tuple)) {
    auto hort_plane = hort_planes.find(hort_plane_id);
    this->fill_plane_points<HorizontalPlanes>(hort_plane->second);
  }
}

template <typename T>
void PlaneMapper::fill_plane_points(T& plane) {
  plane.cloud_seg_map->points.clear();

  for (int k = 0; k < plane.keyframe_node_vec.size(); ++k) {
    bool marginalized = GraphUtils::get_keyframe_marg_data(plane.keyframe_node_vec[k]);

    if (marginalized) continue;

    Eigen::Matrix4f pose =
        plane.keyframe_node_vec[k]->estimate().matrix().template cast<float>();

    for (size_t j = 0; j < plane.cloud_seg_body_vec[k]->points.size(); ++j) {
      PointNormal dst_pt;
      dst_pt.getVector4fMap() =
          pose * plane.cloud_seg_body_vec[k]->points[j].getVector4fMap();
      dst_pt.r = plane.color[0];
      dst_pt.g = plane.color[1];
      dst_pt.b = plane.color[2];
      dst_pt.a = plane.color[3];
      plane.cloud_seg_map->points.push_back(dst_pt);
    }
  }
}

}  // namespace s_graphs
