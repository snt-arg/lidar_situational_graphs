/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

// SPDX-License-Identifier: BSD-2-Clause

#include "s_graphs/common/plane_utils.hpp"

namespace s_graphs {

PlaneUtils::PlaneUtils() {}

float PlaneUtils::width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2) {
  Eigen::Vector3d vec;
  float size = 0;

  if (fabs(v1(3)) > fabs(v2(3)))
    vec = fabs(v1(3)) * v1.head(3) - fabs(v2(3)) * v2.head(3);
  else if (fabs(v2(3)) > fabs(v1(3)))
    vec = fabs(v2(3)) * v2.head(3) - fabs(v1(3)) * v1.head(3);

  size = fabs(vec(0) + vec(1));

  return size;
}

float PlaneUtils::width_between_planes(s_graphs::msg::PlaneData& plane1,
                                       s_graphs::msg::PlaneData& plane2) {
  Eigen::Vector3d vec;
  Eigen::Vector3d plane1_eigen, plane2_eigen;
  plane1_eigen << plane1.nx, plane1.ny, plane1.nz;
  plane2_eigen << plane2.nx, plane2.ny, plane2.nz;
  float size = 0;

  if (fabs(plane1.d) > fabs(plane2.d))
    vec = fabs(plane1.d) * plane1_eigen - fabs(plane2.d) * plane2_eigen;
  else if (fabs(plane2.d) > fabs(plane1.d))
    vec = fabs(plane2.d) * plane2_eigen - fabs(plane1.d) * plane1_eigen;

  size = fabs(vec(0) + vec(1));

  return size;
}

void PlaneUtils::correct_plane_direction(int plane_type,
                                         s_graphs::msg::PlaneData& plane) {
  if (plane.d > 0) {
    plane.nx = -1 * plane.nx;
    plane.ny = -1 * plane.ny;
    plane.nz = -1 * plane.nz;
    plane.d = -1 * plane.d;
  }
  return;
}

void PlaneUtils::correct_plane_direction(int plane_type, Eigen::Vector4d& plane) {
  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  }
  return;
}

Eigen::Quaterniond PlaneUtils::euler_to_quaternion(const double roll,
                                                   const double pitch,
                                                   const double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

Eigen::Vector2d PlaneUtils::room_center(const Eigen::Vector4d& x_plane1,
                                        const Eigen::Vector4d& x_plane2,
                                        const Eigen::Vector4d& y_plane1,
                                        const Eigen::Vector4d& y_plane2) {
  Eigen::Vector2d center;
  Eigen::Vector3d vec_x, vec_y;

  if (fabs(x_plane1(3)) > fabs(x_plane2(3))) {
    vec_x = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) -
                    fabs(x_plane2(3)) * x_plane2.head(3))) +
            fabs(x_plane2(3)) * x_plane2.head(3);
  } else {
    vec_x = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) -
                    fabs(x_plane1(3)) * x_plane1.head(3))) +
            fabs(x_plane1(3)) * x_plane1.head(3);
  }

  if (fabs(y_plane1(3)) > fabs(y_plane2(3))) {
    vec_y = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) -
                    fabs(y_plane2(3)) * y_plane2.head(3))) +
            fabs(y_plane2(3)) * y_plane2.head(3);
  } else {
    vec_y = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) -
                    fabs(y_plane1(3)) * y_plane1.head(3))) +
            fabs(y_plane1(3)) * y_plane1.head(3);
  }

  Eigen::Vector3d final_vec = vec_x + vec_y;
  center(0) = final_vec(0);
  center(1) = final_vec(1);

  return center;
}

geometry_msgs::msg::Pose PlaneUtils::room_center(
    const s_graphs::msg::PlaneData& x_plane1,
    const s_graphs::msg::PlaneData& x_plane2,
    const s_graphs::msg::PlaneData& y_plane1,
    const s_graphs::msg::PlaneData& y_plane2) {
  geometry_msgs::msg::Pose center;
  Eigen::Vector3d vec_x, vec_y;
  Eigen::Vector3d x_plane1_eigen, x_plane2_eigen;
  Eigen::Vector3d y_plane1_eigen, y_plane2_eigen;

  x_plane1_eigen << x_plane1.nx, x_plane1.ny, x_plane1.nz;
  x_plane2_eigen << x_plane2.nx, x_plane2.ny, x_plane2.nz;

  y_plane1_eigen << y_plane1.nx, y_plane1.ny, y_plane1.nz;
  y_plane2_eigen << y_plane2.nx, y_plane2.ny, y_plane2.nz;

  if (fabs(x_plane1.d) > fabs(x_plane2.d)) {
    vec_x = (0.5 *
             (fabs(x_plane1.d) * x_plane1_eigen - fabs(x_plane2.d) * x_plane2_eigen)) +
            fabs(x_plane2.d) * x_plane2_eigen;
  } else {
    vec_x = (0.5 *
             (fabs(x_plane2.d) * x_plane2_eigen - fabs(x_plane1.d) * x_plane1_eigen)) +
            fabs(x_plane1.d) * x_plane1_eigen;
  }

  if (fabs(y_plane1.d) > fabs(y_plane2.d)) {
    vec_y = (0.5 *
             (fabs(y_plane1.d) * y_plane1_eigen - fabs(y_plane2.d) * y_plane2_eigen)) +
            fabs(y_plane2.d) * y_plane2_eigen;
  } else {
    vec_y = (0.5 *
             (fabs(y_plane2.d) * y_plane2_eigen - fabs(y_plane1.d) * y_plane1_eigen)) +
            fabs(y_plane1.d) * y_plane1_eigen;
  }

  Eigen::Vector3d final_vec = vec_x + vec_y;
  center.position.x = final_vec(0);
  center.position.y = final_vec(1);
  center.position.z = final_vec(2);

  Eigen::Matrix3d room_orientation;
  double yaw, pitch;
  Eigen::Vector3d x_plane1_orietation;
  x_plane1_orietation << x_plane1.plane_orientation.x, x_plane1.plane_orientation.y,
      x_plane1.plane_orientation.z;
  yaw = std::atan2(x_plane1_orietation(1), x_plane1_orietation(1));
  pitch = std::atan2(x_plane1_orietation(2), x_plane1_orietation.head<2>().norm());
  Eigen::Quaterniond room_quat = euler_to_quaternion(0.0, pitch, yaw);

  center.orientation.x = room_quat.x();
  center.orientation.y = room_quat.y();
  center.orientation.z = room_quat.z();
  center.orientation.w = room_quat.w();

  return center;
}

float PlaneUtils::plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg,
                               pcl::PointXY& p1,
                               pcl::PointXY& p2,
                               g2o::VertexSE3* keyframe_node) {
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

float PlaneUtils::plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg,
                               pcl::PointXY& p1,
                               pcl::PointXY& p2) {
  PointNormal pmin, pmax;
  pcl::getMaxSegment(*cloud_seg, pmin, pmax);
  p1.x = pmin.x;
  p1.y = pmin.y;
  p2.x = pmax.x;
  p2.y = pmax.y;
  float length = pcl::euclideanDistance(p1, p2);

  return length;
}

pcl::PointXY PlaneUtils::convert_point_to_map(pcl::PointXY point_local,
                                              Eigen::Matrix4d keyframe_pose) {
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

float PlaneUtils::get_min_segment(const pcl::PointCloud<PointNormal>::Ptr& cloud_1,
                                  const pcl::PointCloud<PointNormal>::Ptr& cloud_2) {
  float min_dist = std::numeric_limits<float>::max();
  const auto token = std::numeric_limits<std::size_t>::max();
  std::size_t i_min = token, i_max = token;

  for (std::size_t i = 0; i < cloud_1->points.size(); ++i) {
    for (std::size_t j = 0; j < cloud_2->points.size(); ++j) {
      // Compute the distance
      float dist =
          (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap())
              .squaredNorm();
      if (dist >= min_dist) continue;

      min_dist = dist;
      i_min = i;
      i_max = j;
    }
  }

  return (std::sqrt(min_dist));
}

bool PlaneUtils::check_point_neighbours(
    const pcl::PointCloud<PointNormal>::Ptr& cloud_1,
    const pcl::PointCloud<PointNormal>::Ptr& cloud_2) {
  bool valid_neighbour = false;
  int point_count = 0;
  float min_dist = 0.5;

  for (std::size_t i = 0; i < cloud_1->points.size(); ++i) {
    for (std::size_t j = 0; j < cloud_2->points.size(); ++j) {
      // Compute the distance
      float dist =
          (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap())
              .squaredNorm();
      if (dist < min_dist) {
        point_count++;
        break;
      }
    }
    if (point_count > 100) {
      valid_neighbour = true;
      break;
    }
  }
  return valid_neighbour;
}

bool PlaneUtils::compute_point_difference(const double plane1_point,
                                          const double plane2_point) {
  if ((plane1_point - plane2_point) > 0) return false;

  return true;
}

float PlaneUtils::plane_dot_product(const s_graphs::msg::PlaneData& plane1,
                                    const s_graphs::msg::PlaneData& plane2) {
  float dot_product =
      plane1.nx * plane2.nx + plane1.ny * plane2.ny + plane1.nz * plane2.nz;
  return dot_product;
}

bool PlaneUtils::plane_dot_product(g2o::VertexPlane* plane1, g2o::VertexPlane* plane2) {
  Eigen::Vector4d coeffs1 = plane1->estimate().coeffs();
  Eigen::Vector4d coeffs2 = plane2->estimate().coeffs();
  Eigen::Vector3d normal1(coeffs1[0], coeffs1[1], coeffs1[2]);
  Eigen::Vector3d normal2(coeffs2[0], coeffs2[1], coeffs2[2]);
  double dot_product = normal1.dot(normal2);
  std::cout << "dot product : " << dot_product << std::endl;
  return dot_product > 0.9;
}

geometry_msgs::msg::Pose PlaneUtils::extract_infite_room_center(
    int plane_type,
    pcl::PointXY p1,
    pcl::PointXY p2,
    s_graphs::msg::PlaneData plane1,
    s_graphs::msg::PlaneData plane2,
    Eigen::Vector2d& cluster_center) {
  geometry_msgs::msg::Pose center_point;
  Eigen::Vector4d plane1_eigen, plane2_eigen;
  plane1_eigen << plane1.nx, plane1.ny, plane1.nz, plane1.d;
  plane2_eigen << plane2.nx, plane2.ny, plane2.nz, plane2.d;

  if (fabs(p1.x) > fabs(p2.x)) {
    float size = p1.x - p2.x;
    cluster_center(0) = (size / 2) + p2.x;
  } else {
    float size = p2.x - p1.x;
    cluster_center(0) = (size / 2) + p1.x;
  }

  if (fabs(p1.y) > fabs(p2.y)) {
    float size = p1.y - p2.y;
    cluster_center(1) = (size / 2) + p2.y;
  } else {
    float size = p2.y - p1.y;
    cluster_center(1) = (size / 2) + p1.y;
  }

  Eigen::Vector3d vec;
  Eigen::Vector2d vec_normal, final_pose_vec;

  if (fabs(plane1_eigen(3)) > fabs(plane2_eigen(3))) {
    vec = (0.5 * (fabs(plane1_eigen(3)) * plane1_eigen.head(3) -
                  fabs(plane2_eigen(3)) * plane2_eigen.head(3))) +
          fabs(plane2_eigen(3)) * plane2_eigen.head(3);
  } else {
    vec = (0.5 * (fabs(plane2_eigen(3)) * plane2_eigen.head(3) -
                  fabs(plane1_eigen(3)) * plane1_eigen.head(3))) +
          fabs(plane1_eigen(3)) * plane1_eigen.head(3);
  }

  vec_normal = vec.head(2) / vec.norm();
  final_pose_vec =
      vec.head(2) + (cluster_center - (cluster_center.dot(vec_normal)) * vec_normal);
  center_point.position.x = final_pose_vec(0);
  center_point.position.y = final_pose_vec(1);

  return center_point;
}

double PlaneUtils::plane_difference(g2o::Plane3D plane1, g2o::Plane3D plane2) {
  Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
  Eigen::Vector3d error = plane1.ominus(plane2);
  double maha_dist = sqrt(error.transpose() * information * error);
  return maha_dist;
}

void PlaneUtils::get_start_and_end_points(VerticalPlanes& plane) {
  if (plane.cloud_seg_map->empty()) {
    std::cerr << "Point cloud is empty." << std::endl;
  }

  // Initialize variables to store the closest and farthest points
  auto closest_point = plane.cloud_seg_map->points[0];
  auto farthest_point = plane.cloud_seg_map->points[0];

  double min_distance =
      std::sqrt(closest_point.x * closest_point.x + closest_point.y * closest_point.y +
                closest_point.z * closest_point.z);

  double max_distance = min_distance;

  // Loop through the point cloud
  for (const auto& point : plane.cloud_seg_map->points) {
    // Calculate the Euclidean distance from the origin (0, 0, 0)
    double distance =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    // Update closest and farthest points
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
    }

    if (distance > max_distance) {
      max_distance = distance;
      farthest_point = point;
    }
  }

  // Output the coordinates of the closest and farthest points
  plane.starting_point = closest_point;
  plane.ending_point = farthest_point;
  // std::cout << "Closest Point: (" << plane.starting_point.x << ", "
  //           << plane.starting_point.y << "," << plane.starting_point.z << ")\n";
  // std::cout << "Farthest Point: (" << farthest_point.x << ", " << farthest_point.y
  //           << ", " << farthest_point.z << ")\n";
}

}  // namespace s_graphs
