// SPDX-License-Identifier: BSD-2-Clause

#ifndef PLANE_UTILS_HPP
#define PLANE_UTILS_HPP

#include <Eigen/Dense>

#include <s_graphs/PlanesData.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>

#include <geometry_msgs/Point.h>

namespace s_graphs {

enum plane_class : uint8_t {
  X_VERT_PLANE = 0,
  Y_VERT_PLANE = 1,
  HORT_PLANE = 2,
};

/**
 * @brief
 */
class PlaneUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  PlaneUtils() {}
  ~PlaneUtils() {}

  /**
   * @brief
   *
   * @param
   * @return
   */
  inline float width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2) {
    Eigen::Vector3d vec;
    float size = 0;

    if(fabs(v1(3)) > fabs(v2(3)))
      vec = fabs(v1(3)) * v1.head(3) - fabs(v2(3)) * v2.head(3);
    else if(fabs(v2(3)) > fabs(v1(3)))
      vec = fabs(v2(3)) * v2.head(3) - fabs(v1(3)) * v1.head(3);

    size = fabs(vec(0) + vec(1));

    return size;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  inline float width_between_planes(s_graphs::PlaneData& plane1, s_graphs::PlaneData& plane2) {
    Eigen::Vector3d vec;
    Eigen::Vector3d plane1_eigen, plane2_eigen;
    plane1_eigen << plane1.nx, plane1.ny, plane1.nz;
    plane2_eigen << plane2.nx, plane2.ny, plane2.nz;
    float size = 0;

    if(fabs(plane1.d) > fabs(plane2.d))
      vec = fabs(plane1.d) * plane1_eigen - fabs(plane2.d) * plane2_eigen;
    else if(fabs(plane2.d) > fabs(plane1.d))
      vec = fabs(plane2.d) * plane2_eigen - fabs(plane1.d) * plane1_eigen;

    size = fabs(vec(0) + vec(1));

    return size;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  void correct_plane_direction(int plane_type, s_graphs::PlaneData& plane) {
    if(plane.d > 0) {
      plane.nx = -1 * plane.nx;
      plane.ny = -1 * plane.ny;
      plane.nz = -1 * plane.nz;
      plane.d = -1 * plane.d;
    }
    return;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  void correct_plane_direction(int plane_type, Eigen::Vector4d& plane) {
    if(plane(3) > 0) {
      plane(0) = -1 * plane(0);
      plane(1) = -1 * plane(1);
      plane(2) = -1 * plane(2);
      plane(3) = -1 * plane(3);
    }
    return;
  }

  void correct_plane_d(int plane_type, s_graphs::PlaneData& plane) {
    if(plane_type == plane_class::X_VERT_PLANE) {
      plane.d = -1 * plane.d;
      double p_norm = plane.nx / fabs(plane.nx);
      plane.d = p_norm * plane.d;
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      plane.d = -1 * plane.d;
      double p_norm = plane.ny / fabs(plane.ny);
      plane.d = p_norm * plane.d;
    }
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  Eigen::Vector2d room_center(const Eigen::Vector4d& x_plane1, const Eigen::Vector4d& x_plane2, const Eigen::Vector4d& y_plane1, const Eigen::Vector4d& y_plane2) {
    Eigen::Vector2d center;
    Eigen::Vector3d vec_x, vec_y;

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
    center(0) = final_vec(0);
    center(1) = final_vec(1);

    return center;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  geometry_msgs::Point room_center(const s_graphs::PlaneData& x_plane1, const s_graphs::PlaneData& x_plane2, const s_graphs::PlaneData& y_plane1, const s_graphs::PlaneData& y_plane2) {
    geometry_msgs::Point center;
    Eigen::Vector3d vec_x, vec_y;
    Eigen::Vector3d x_plane1_eigen, x_plane2_eigen;
    Eigen::Vector3d y_plane1_eigen, y_plane2_eigen;

    x_plane1_eigen << x_plane1.nx, x_plane1.ny, x_plane1.nz;
    x_plane2_eigen << x_plane2.nx, x_plane2.ny, x_plane2.nz;

    y_plane1_eigen << y_plane1.nx, y_plane1.ny, y_plane1.nz;
    y_plane2_eigen << y_plane2.nx, y_plane2.ny, y_plane2.nz;

    if(fabs(x_plane1.d) > fabs(x_plane2.d)) {
      vec_x = (0.5 * (fabs(x_plane1.d) * x_plane1_eigen - fabs(x_plane2.d) * x_plane2_eigen)) + fabs(x_plane2.d) * x_plane2_eigen;
    } else {
      vec_x = (0.5 * (fabs(x_plane2.d) * x_plane2_eigen - fabs(x_plane1.d) * x_plane1_eigen)) + fabs(x_plane1.d) * x_plane1_eigen;
    }

    if(fabs(y_plane1.d) > fabs(y_plane2.d)) {
      vec_y = (0.5 * (fabs(y_plane1.d) * y_plane1_eigen - fabs(y_plane2.d) * y_plane2_eigen)) + fabs(y_plane2.d) * y_plane2_eigen;
    } else {
      vec_y = (0.5 * (fabs(y_plane2.d) * y_plane2_eigen - fabs(y_plane1.d) * y_plane1_eigen)) + fabs(y_plane1.d) * y_plane1_eigen;
    }

    Eigen::Vector3d final_vec = vec_x + vec_y;
    center.x = final_vec(0);
    center.y = final_vec(1);

    return center;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg, pcl::PointXY& p1, pcl::PointXY& p2, g2o::VertexSE3* keyframe_node) {
    PointNormal pmin, pmax;
    pcl::getMaxSegment(*cloud_seg, pmin, pmax);
    p1.x = pmin.x;
    p1.y = pmin.y;
    p2.x = pmax.x;
    p2.y = pmax.y;
    float length = pcl::euclideanDistance(p1, p2);

    pcl::PointXY p1_map, p2_map;
    p1_map = convert_point_to_map(p1, keyframe_node->estimate().matrix());
    p2_map = convert_point_to_map(p2, keyframe_node->estimate().matrix());
    p1 = p1_map;
    p2 = p2_map;

    return length;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg, pcl::PointXY& p1, pcl::PointXY& p2) {
    PointNormal pmin, pmax;
    pcl::getMaxSegment(*cloud_seg, pmin, pmax);
    p1.x = pmin.x;
    p1.y = pmin.y;
    p2.x = pmax.x;
    p2.y = pmax.y;
    float length = pcl::euclideanDistance(p1, p2);

    return length;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointXY convert_point_to_map(pcl::PointXY point_local, Eigen::Matrix4d keyframe_pose) {
    pcl::PointXY point_map;

    Eigen::Vector4d point_map_eigen, point_local_eigen;
    point_local_eigen = point_map_eigen.setZero();
    point_local_eigen(3) = point_map_eigen(3) = 1;
    point_local_eigen(0) = point_local.x;
    point_local_eigen(1) = point_local.y;
    point_map_eigen = keyframe_pose * point_local_eigen;

    point_map.x = point_map_eigen(0);
    point_map.y = point_map_eigen(1);
    return point_map;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  float get_min_segment(const pcl::PointCloud<PointNormal>::Ptr& cloud_1, const pcl::PointCloud<PointNormal>::Ptr& cloud_2) {
    float min_dist = std::numeric_limits<float>::max();
    const auto token = std::numeric_limits<std::size_t>::max();
    std::size_t i_min = token, i_max = token;

    for(std::size_t i = 0; i < cloud_1->points.size(); ++i) {
      for(std::size_t j = 0; j < cloud_2->points.size(); ++j) {
        // Compute the distance
        float dist = (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap()).squaredNorm();
        if(dist >= min_dist) continue;

        min_dist = dist;
        i_min = i;
        i_max = j;
      }
    }

    return (std::sqrt(min_dist));
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_point_neighbours(const pcl::PointCloud<PointNormal>::Ptr& cloud_1, const pcl::PointCloud<PointNormal>::Ptr& cloud_2) {
    bool valid_neighbour = false;
    int point_count = 0;
    float min_dist = 0.5;

    for(std::size_t i = 0; i < cloud_1->points.size(); ++i) {
      for(std::size_t j = 0; j < cloud_2->points.size(); ++j) {
        // Compute the distance
        float dist = (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap()).squaredNorm();
        if(dist < min_dist) {
          point_count++;
          break;
        }
      }
      if(point_count > 100) {
        valid_neighbour = true;
        break;
      }
    }
    return valid_neighbour;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool compute_point_difference(const double plane1_point, const double plane2_point) {
    if((plane1_point - plane2_point) > 0) return false;

    return true;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  float plane_dot_product(const s_graphs::PlaneData& plane1, const s_graphs::PlaneData& plane2) {
    float dot_product = plane1.nx * plane2.nx + plane1.ny * plane2.ny + plane1.nz * plane2.nz;
    return dot_product;
  }

  geometry_msgs::Point extract_infite_room_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2, Eigen::Vector2d& cluster_center) {
    geometry_msgs::Point center_point;
    Eigen::Vector4d plane1_eigen, plane2_eigen;
    plane1_eigen << plane1.nx, plane1.ny, plane1.nz, plane1.d;
    plane2_eigen << plane2.nx, plane2.ny, plane2.nz, plane2.d;

    if(fabs(p1.x) > fabs(p2.x)) {
      float size = p1.x - p2.x;
      cluster_center(0) = (size / 2) + p2.x;
    } else {
      float size = p2.x - p1.x;
      cluster_center(0) = (size / 2) + p1.x;
    }

    if(fabs(p1.y) > fabs(p2.y)) {
      float size = p1.y - p2.y;
      cluster_center(1) = (size / 2) + p2.y;
    } else {
      float size = p2.y - p1.y;
      cluster_center(1) = (size / 2) + p1.y;
    }

    Eigen::Vector3d vec;
    Eigen::Vector2d vec_normal, final_pose_vec;

    if(fabs(plane1_eigen(3)) > fabs(plane2_eigen(3))) {
      vec = (0.5 * (fabs(plane1_eigen(3)) * plane1_eigen.head(3) - fabs(plane2_eigen(3)) * plane2_eigen.head(3))) + fabs(plane2_eigen(3)) * plane2_eigen.head(3);
    } else {
      vec = (0.5 * (fabs(plane2_eigen(3)) * plane2_eigen.head(3) - fabs(plane1_eigen(3)) * plane1_eigen.head(3))) + fabs(plane1_eigen(3)) * plane1_eigen.head(3);
    }

    vec_normal = vec.head(2) / vec.norm();
    final_pose_vec = vec.head(2) + (cluster_center - (cluster_center.dot(vec_normal)) * vec_normal);
    center_point.x = final_pose_vec(0);
    center_point.y = final_pose_vec(1);

    return center_point;
  }
  double plane_difference(g2o::Plane3D plane1, g2o::Plane3D plane2) {
    Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
    Eigen::Vector3d error = plane1.ominus(plane2);
    double maha_dist = sqrt(error.transpose() * information * error);
    return maha_dist;
  }
};
}  // namespace s_graphs
#endif  // PLANE_UTILS_HPP
