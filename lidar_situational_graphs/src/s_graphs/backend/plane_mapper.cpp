#include <s_graphs/backend/plane_mapper.hpp>

namespace s_graphs {

PlaneMapper::PlaneMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
  node_obj = node;
  use_point_to_plane =
      node_obj->get_parameter("use_point_to_plane").get_parameter_value().get<bool>();
  plane_information =
      node_obj->get_parameter("plane_information").get_parameter_value().get<double>();
  plane_dist_threshold = node_obj->get_parameter("plane_dist_threshold")
                             .get_parameter_value()
                             .get<double>();
  plane_points_dist =
      node_obj->get_parameter("plane_points_dist").get_parameter_value().get<double>();
  min_plane_points =
      node_obj->get_parameter("min_plane_points").get_parameter_value().get<int>();
}

PlaneMapper::~PlaneMapper() {}

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

  int data_association;
  data_association = -1;
  data_association = associate_plane(plane_type,
                                     keyframe,
                                     det_plane_body_frame.coeffs(),
                                     cloud_seg_body,
                                     x_vert_planes,
                                     y_vert_planes,
                                     hort_planes);

  switch (plane_type) {
    case PlaneUtils::plane_class::X_VERT_PLANE: {
      if (x_vert_planes.empty() || data_association == -1) {
        data_association = covisibility_graph->retrieve_local_nbr_of_vertices();
        plane_node = covisibility_graph->add_plane_node(det_plane_map_frame.coeffs());
        VerticalPlanes vert_plane;
        vert_plane.id = data_association;
        vert_plane.cloud_seg_body_vec.push_back(cloud_seg_body);
        vert_plane.keyframe_node_vec.push_back(keyframe->node);
        vert_plane.plane_node = plane_node;
        vert_plane.cloud_seg_map =
            pcl::PointCloud<PointNormal>::Ptr(new pcl::PointCloud<PointNormal>);
        vert_plane.covariance = Eigen::Matrix3d::Identity();
        vert_plane.floor_level = keyframe->floor_level;
        std::vector<double> color;
        color.push_back(cloud_seg_body->points.back().r);  // red
        color.push_back(cloud_seg_body->points.back().g);  // green
        color.push_back(cloud_seg_body->points.back().b);  // blue
        vert_plane.color = color;

        shared_graph_mutex.lock();
        x_vert_planes.insert({vert_plane.id, vert_plane});
        keyframe->x_plane_ids.push_back(vert_plane.id);
        shared_graph_mutex.unlock();
      } else {
        shared_graph_mutex.lock();
        plane_node = x_vert_planes[data_association].plane_node;
        x_vert_planes[data_association].cloud_seg_body_vec.push_back(cloud_seg_body);
        x_vert_planes[data_association].keyframe_node_vec.push_back(keyframe->node);
        keyframe->x_plane_ids.push_back(x_vert_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
    case PlaneUtils::plane_class::Y_VERT_PLANE: {
      if (y_vert_planes.empty() || data_association == -1) {
        data_association = covisibility_graph->retrieve_local_nbr_of_vertices();
        plane_node = covisibility_graph->add_plane_node(det_plane_map_frame.coeffs());
        VerticalPlanes vert_plane;
        vert_plane.id = data_association;
        vert_plane.cloud_seg_body_vec.push_back(cloud_seg_body);
        vert_plane.keyframe_node_vec.push_back(keyframe->node);
        vert_plane.plane_node = plane_node;
        vert_plane.cloud_seg_map =
            pcl::PointCloud<PointNormal>::Ptr(new pcl::PointCloud<PointNormal>);
        vert_plane.covariance = Eigen::Matrix3d::Identity();
        vert_plane.floor_level = keyframe->floor_level;
        std::vector<double> color;
        color.push_back(cloud_seg_body->points.back().r);  // red
        color.push_back(cloud_seg_body->points.back().g);  // green
        color.push_back(cloud_seg_body->points.back().b);  // blue
        vert_plane.color = color;

        shared_graph_mutex.lock();
        y_vert_planes.insert({vert_plane.id, vert_plane});
        keyframe->y_plane_ids.push_back(vert_plane.id);
        shared_graph_mutex.unlock();

      } else {
        shared_graph_mutex.lock();
        plane_node = y_vert_planes[data_association].plane_node;
        y_vert_planes[data_association].cloud_seg_body_vec.push_back(cloud_seg_body);
        y_vert_planes[data_association].keyframe_node_vec.push_back(keyframe->node);
        keyframe->y_plane_ids.push_back(y_vert_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
    case PlaneUtils::plane_class::HORT_PLANE: {
      if (hort_planes.empty() || data_association == -1) {
        data_association = covisibility_graph->retrieve_local_nbr_of_vertices();
        plane_node = covisibility_graph->add_plane_node(det_plane_map_frame.coeffs());
        HorizontalPlanes hort_plane;
        hort_plane.id = data_association;
        hort_plane.cloud_seg_body_vec.push_back(cloud_seg_body);
        hort_plane.keyframe_node_vec.push_back(keyframe->node);
        hort_plane.plane_node = plane_node;
        hort_plane.cloud_seg_map =
            pcl::PointCloud<PointNormal>::Ptr(new pcl::PointCloud<PointNormal>);
        hort_plane.covariance = Eigen::Matrix3d::Identity();
        hort_plane.floor_level = keyframe->floor_level;
        std::vector<double> color;
        color.push_back(255);  // red
        color.push_back(0.0);  // green
        color.push_back(100);  // blue
        hort_plane.color = color;

        shared_graph_mutex.lock();
        hort_planes.insert({hort_plane.id, hort_plane});
        keyframe->hort_plane_ids.push_back(hort_plane.id);
        shared_graph_mutex.unlock();

      } else {
        shared_graph_mutex.lock();
        plane_node = hort_planes[data_association].plane_node;
        hort_planes[data_association].cloud_seg_body_vec.push_back(cloud_seg_body);
        hort_planes[data_association].keyframe_node_vec.push_back(keyframe->node);
        keyframe->hort_plane_ids.push_back(hort_planes[data_association].id);
        shared_graph_mutex.unlock();
      }
      break;
    }
    default:
      std::cout << "factoring planes function had a weird error " << std::endl;
      break;
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

    auto edge = covisibility_graph->add_se3_plane_edge(keyframe->node,
                                                       plane_node,
                                                       det_plane_body_frame.coeffs(),
                                                       plane_information_mat);
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
  }

  convert_plane_points_to_map(
      x_vert_planes, y_vert_planes, hort_planes, keyframe->floor_level);
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
  int data_association;
  double vert_min_maha_dist = 100;
  double hort_min_maha_dist = 100;

  shared_graph_mutex.lock();
  Eigen::Isometry3d m2n = keyframe->estimate().inverse();
  shared_graph_mutex.unlock();

  int current_floor_level = keyframe->floor_level;

  switch (plane_type) {
    case PlaneUtils::plane_class::X_VERT_PLANE: {
      for (const auto& x_vert_plane : x_vert_planes) {
        if (current_floor_level != x_vert_plane.second.floor_level) continue;

        shared_graph_mutex.lock();
        g2o::Plane3D local_plane = m2n * x_vert_plane.second.plane_node->estimate();
        shared_graph_mutex.unlock();

        Eigen::Vector3d error = local_plane.ominus(det_plane);
        double maha_dist =
            sqrt(error.transpose() * x_vert_plane.second.covariance.inverse() * error);

        if (std::isnan(maha_dist) || maha_dist < 1e-3) {
          Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
          maha_dist = sqrt(error.transpose() * cov * error);
        }
        if (maha_dist < vert_min_maha_dist) {
          vert_min_maha_dist = maha_dist;
          data_association = x_vert_plane.second.id;
        }
      }
      if (vert_min_maha_dist < plane_dist_threshold) {
        if (!x_vert_planes.at(data_association).cloud_seg_map->empty()) {
          float min_segment = std::numeric_limits<float>::max();
          pcl::PointCloud<PointNormal>::Ptr cloud_seg_detected(
              new pcl::PointCloud<PointNormal>());

          shared_graph_mutex.lock();
          Eigen::Matrix4f current_keyframe_pose =
              keyframe->estimate().matrix().cast<float>();
          shared_graph_mutex.unlock();

          for (size_t j = 0; j < cloud_seg_body->points.size(); ++j) {
            PointNormal dst_pt;
            dst_pt.getVector4fMap() =
                current_keyframe_pose * cloud_seg_body->points[j].getVector4fMap();
            cloud_seg_detected->points.push_back(dst_pt);
          }
          bool valid_neighbour = PlaneUtils::check_point_neighbours(
              x_vert_planes.at(data_association).cloud_seg_map, cloud_seg_detected);

          if (!valid_neighbour) {
            data_association = -1;
          }
        }
      } else {
        data_association = -1;
      }
      break;
    }
    case PlaneUtils::plane_class::Y_VERT_PLANE: {
      for (const auto& y_vert_plane : y_vert_planes) {
        if (current_floor_level != y_vert_plane.second.floor_level) continue;

        shared_graph_mutex.lock();
        g2o::Plane3D local_plane = m2n * y_vert_plane.second.plane_node->estimate();
        shared_graph_mutex.unlock();

        Eigen::Vector3d error = local_plane.ominus(det_plane);
        double maha_dist =
            sqrt(error.transpose() * y_vert_plane.second.covariance.inverse() * error);

        if (std::isnan(maha_dist) || maha_dist < 1e-3) {
          Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
          maha_dist = sqrt(error.transpose() * cov * error);
        }
        if (maha_dist < vert_min_maha_dist) {
          vert_min_maha_dist = maha_dist;
          data_association = y_vert_plane.second.id;
        }
      }
      if (vert_min_maha_dist < plane_dist_threshold) {
        if (!y_vert_planes.at(data_association).cloud_seg_map->empty()) {
          float min_segment = std::numeric_limits<float>::max();
          pcl::PointCloud<PointNormal>::Ptr cloud_seg_detected(
              new pcl::PointCloud<PointNormal>());

          shared_graph_mutex.lock();
          Eigen::Matrix4f current_keyframe_pose =
              keyframe->estimate().matrix().cast<float>();
          shared_graph_mutex.unlock();

          for (size_t j = 0; j < cloud_seg_body->points.size(); ++j) {
            PointNormal dst_pt;
            dst_pt.getVector4fMap() =
                current_keyframe_pose * cloud_seg_body->points[j].getVector4fMap();
            cloud_seg_detected->points.push_back(dst_pt);
          }
          bool valid_neighbour = PlaneUtils::check_point_neighbours(
              y_vert_planes.at(data_association).cloud_seg_map, cloud_seg_detected);

          if (!valid_neighbour) {
            data_association = -1;
          }
        }
      } else {
        data_association = -1;
      }
      break;
    }
    case PlaneUtils::plane_class::HORT_PLANE: {
      for (const auto& hort_plane : hort_planes) {
        if (current_floor_level != hort_plane.second.floor_level) continue;

        shared_graph_mutex.lock();
        g2o::Plane3D local_plane = m2n * hort_plane.second.plane_node->estimate();
        shared_graph_mutex.unlock();

        Eigen::Vector3d error = local_plane.ominus(det_plane);
        double maha_dist =
            sqrt(error.transpose() * hort_plane.second.covariance.inverse() * error);

        if (std::isnan(maha_dist) || maha_dist < 1e-3) {
          Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
          maha_dist = sqrt(error.transpose() * cov * error);
        }
        if (maha_dist < hort_min_maha_dist) {
          hort_min_maha_dist = maha_dist;
          data_association = hort_plane.second.id;
        }
      }

      if (hort_min_maha_dist > plane_dist_threshold) data_association = -1;
      break;
    }
    default:
      std::cout << "associating planes had an error " << std::endl;
      break;
  }

  // printf("\n vert min maha dist: %f", vert_min_maha_dist);
  // printf("\n hort min maha dist: %f", hort_min_maha_dist);

  return data_association;
}

/**
 * @brief convert the body points of planes to map frame for mapping
 */
void PlaneMapper::convert_plane_points_to_map(
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, HorizontalPlanes>& hort_planes,
    const int& current_floor_level) {
  for (auto& x_vert_plane : x_vert_planes) {
    if (x_vert_plane.second.floor_level != current_floor_level) break;

    x_vert_plane.second.cloud_seg_map->points.clear();
    for (int k = 0; k < x_vert_plane.second.keyframe_node_vec.size(); ++k) {
      bool marginalized =
          GraphUtils::get_keyframe_marg_data(x_vert_plane.second.keyframe_node_vec[k]);
      if (marginalized) continue;

      Eigen::Matrix4f pose =
          x_vert_plane.second.keyframe_node_vec[k]->estimate().matrix().cast<float>();

      for (size_t j = 0; j < x_vert_plane.second.cloud_seg_body_vec[k]->points.size();
           ++j) {
        PointNormal dst_pt;
        dst_pt.getVector4fMap() =
            pose *
            x_vert_plane.second.cloud_seg_body_vec[k]->points[j].getVector4fMap();
        x_vert_plane.second.cloud_seg_map->points.push_back(dst_pt);
      }
    }
  }

  for (auto& y_vert_plane : y_vert_planes) {
    if (y_vert_plane.second.floor_level != current_floor_level) break;

    y_vert_plane.second.cloud_seg_map->points.clear();
    for (int k = 0; k < y_vert_plane.second.keyframe_node_vec.size(); ++k) {
      bool marginalized =
          GraphUtils::get_keyframe_marg_data(y_vert_plane.second.keyframe_node_vec[k]);
      if (marginalized) continue;

      Eigen::Matrix4f pose =
          y_vert_plane.second.keyframe_node_vec[k]->estimate().matrix().cast<float>();

      for (size_t j = 0; j < y_vert_plane.second.cloud_seg_body_vec[k]->points.size();
           ++j) {
        PointNormal dst_pt;
        dst_pt.getVector4fMap() =
            pose *
            y_vert_plane.second.cloud_seg_body_vec[k]->points[j].getVector4fMap();
        y_vert_plane.second.cloud_seg_map->points.push_back(dst_pt);
      }
    }
  }

  for (auto& hort_plane : hort_planes) {
    if (hort_plane.second.floor_level != current_floor_level) break;

    hort_plane.second.cloud_seg_map->points.clear();
    for (int k = 0; k < hort_plane.second.keyframe_node_vec.size(); ++k) {
      bool marginalized =
          GraphUtils::get_keyframe_marg_data(hort_plane.second.keyframe_node_vec[k]);
      if (marginalized) continue;

      Eigen::Matrix4f pose =
          hort_plane.second.keyframe_node_vec[k]->estimate().matrix().cast<float>();

      for (size_t j = 0; j < hort_plane.second.cloud_seg_body_vec[k]->points.size();
           ++j) {
        PointNormal dst_pt;
        dst_pt.getVector4fMap() =
            pose * hort_plane.second.cloud_seg_body_vec[k]->points[j].getVector4fMap();
        hort_plane.second.cloud_seg_map->points.push_back(dst_pt);
      }
    }
  }
}

}  // namespace s_graphs
