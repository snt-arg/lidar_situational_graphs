// SPDX-License-Identifier: BSD-2-Clause

#ifndef PLANE_UTILS_HPP
#define PLANE_UTILS_HPP

#include <Eigen/Dense>
#include <s_graphs/PlanesData.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace s_graphs {

class PlaneUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  PlaneUtils() {}
  ~PlaneUtils() {}

public:
  enum plane_class : uint8_t {
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
    HORT_PLANE = 2,
  };

  inline float width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2) {
    float size = 0;
    if(fabs(v1(3)) > fabs(v2(3)))
      size = fabs(v1(3) - v2(3));
    else if(fabs(v2(3)) > fabs(v1(3)))
      size = fabs(v2(3) - v1(3));

    return size;
  }

  inline float width_between_planes(s_graphs::PlaneData& plane1, s_graphs::PlaneData& plane2) {
    float size = 0;
    float room_width_threshold = 1.0;
    if(fabs(plane1.d) > fabs(plane2.d))
      size = fabs(plane1.d - plane2.d);
    else if(fabs(plane2.d) > fabs(plane1.d))
      size = fabs(plane2.d - plane1.d);

    return size;
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
    return;
  }

  void correct_plane_d(int plane_type, Eigen::Vector4d& plane) {
    if(plane_type == plane_class::X_VERT_PLANE) {
      plane(3) = -1 * plane(3);
      double p_norm = plane(0) / fabs(plane(0));
      plane(3) = p_norm * plane(3);
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      plane(3) = -1 * plane(3);
      double p_norm = plane(1) / fabs(plane(1));
      plane(3) = p_norm * plane(3);
    }
    return;
  }

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

    // if (i_min == token || i_max == token)
    //  return (min_dist = std::numeric_limits<double>::min ());

    // pmin = cloud.points[i_min];
    // pmax = cloud.points[i_max];
    return (std::sqrt(min_dist));
  }
};
}  // namespace s_graphs
#endif  // PLANE_UTILS_HPP
