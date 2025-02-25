#include "s_graphs/common/plane_utils.hpp"

namespace s_graphs {

PlaneUtils::PlaneUtils() {}

float PlaneUtils::width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2) {
  Eigen::Vector3d vec;
  float size = 0;

  if (v1.head(3).dot(v2.head(3)) > 0) return 0;

  correct_plane_direction(v1);
  correct_plane_direction(v2);

  if (fabs(v1(3)) > fabs(v2(3)))
    vec = fabs(v1(3)) * v1.head(3) - fabs(v2(3)) * v2.head(3);
  else if (fabs(v2(3)) > fabs(v1(3)))
    vec = fabs(v2(3)) * v2.head(3) - fabs(v1(3)) * v1.head(3);

  size = fabs(vec(0) + vec(1));

  return size;
}

float PlaneUtils::width_between_planes(
    const situational_graphs_msgs::msg::PlaneData& plane1,
    const situational_graphs_msgs::msg::PlaneData& plane2) {
  Eigen::Vector3d vec;
  Eigen::Vector4d plane1_eigen, plane2_eigen;
  plane1_eigen << plane1.nx, plane1.ny, plane1.nz, plane1.d;
  plane2_eigen << plane2.nx, plane2.ny, plane2.nz, plane2.d;
  float size = 0;

  if (plane1_eigen.head(3).dot(plane2_eigen.head(3)) > 0) return 0;

  correct_plane_direction(plane1_eigen);
  correct_plane_direction(plane2_eigen);

  if (fabs(plane1_eigen(3)) > fabs(plane2_eigen(3)))
    vec = fabs(plane1_eigen(3)) * plane1_eigen.head(3) -
          fabs(plane2_eigen(3)) * plane2_eigen.head(3);
  else if (fabs(plane2_eigen(3)) > fabs(plane1_eigen(3)))
    vec = fabs(plane2_eigen(3)) * plane2_eigen.head(3) -
          fabs(plane1_eigen(3)) * plane1_eigen.head(3);

  size = fabs(vec(0) + vec(1));

  return size;
}

void PlaneUtils::correct_plane_direction(
    situational_graphs_msgs::msg::PlaneData& plane) {
  if (plane.d > 0) {
    plane.nx = -1 * plane.nx;
    plane.ny = -1 * plane.ny;
    plane.nz = -1 * plane.nz;
    plane.d = -1 * plane.d;
  }
  return;
}

void PlaneUtils::correct_plane_direction(Eigen::Vector4d& plane) {
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

geometry_msgs::msg::Pose PlaneUtils::room_center(
    const situational_graphs_msgs::msg::PlaneData& x_plane1,
    const situational_graphs_msgs::msg::PlaneData& x_plane2,
    const situational_graphs_msgs::msg::PlaneData& y_plane1,
    const situational_graphs_msgs::msg::PlaneData& y_plane2) {
  geometry_msgs::msg::Pose center;
  Eigen::Vector3d vec_x, vec_y;
  Eigen::Vector4d x_plane1_eigen, x_plane2_eigen;
  Eigen::Vector4d y_plane1_eigen, y_plane2_eigen;

  x_plane1_eigen << x_plane1.nx, x_plane1.ny, x_plane1.nz, x_plane1.d;
  x_plane2_eigen << x_plane2.nx, x_plane2.ny, x_plane2.nz, x_plane2.d;

  y_plane1_eigen << y_plane1.nx, y_plane1.ny, y_plane1.nz, y_plane1.d;
  y_plane2_eigen << y_plane2.nx, y_plane2.ny, y_plane2.nz, y_plane2.d;

  correct_plane_direction(x_plane1_eigen);
  correct_plane_direction(x_plane2_eigen);

  correct_plane_direction(y_plane1_eigen);
  correct_plane_direction(y_plane2_eigen);

  if (fabs(x_plane1_eigen(3)) > fabs(x_plane2_eigen(3))) {
    vec_x = (0.5 * (fabs(x_plane1_eigen(3)) * x_plane1_eigen.head(3) -
                    fabs(x_plane2_eigen(3)) * x_plane2_eigen.head(3))) +
            fabs(x_plane2_eigen(3)) * x_plane2_eigen.head(3);
  } else {
    vec_x = (0.5 * (fabs(x_plane2_eigen(3)) * x_plane2_eigen.head(3) -
                    fabs(x_plane1_eigen(3)) * x_plane1_eigen.head(3))) +
            fabs(x_plane1_eigen(3)) * x_plane1_eigen.head(3);
  }

  if (fabs(y_plane1_eigen(3)) > fabs(y_plane2_eigen(3))) {
    vec_y = (0.5 * (fabs(y_plane1_eigen(3)) * y_plane1_eigen.head(3) -
                    fabs(y_plane2_eigen(3)) * y_plane2_eigen.head(3))) +
            fabs(y_plane2_eigen(3)) * y_plane2_eigen.head(3);
  } else {
    vec_y = (0.5 * (fabs(y_plane2_eigen(3)) * y_plane2_eigen.head(3) -
                    fabs(y_plane1_eigen(3)) * y_plane1_eigen.head(3))) +
            fabs(y_plane1_eigen(3)) * y_plane1_eigen.head(3);
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
    const pcl::PointCloud<PointNormal>::Ptr& cloud_2,
    const float min_dist) {
  bool valid_neighbour = false;
  int point_count = 0;

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

float PlaneUtils::plane_dot_product(
    const situational_graphs_msgs::msg::PlaneData& plane1,
    const situational_graphs_msgs::msg::PlaneData& plane2) {
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
  return dot_product > 0.9;
}

geometry_msgs::msg::Pose PlaneUtils::extract_infite_room_center(
    int plane_type,
    pcl::PointXY p1,
    pcl::PointXY p2,
    situational_graphs_msgs::msg::PlaneData plane1,
    situational_graphs_msgs::msg::PlaneData plane2,
    Eigen::Vector2d& cluster_center) {
  geometry_msgs::msg::Pose center_point;
  Eigen::Vector4d plane1_eigen, plane2_eigen;
  plane1_eigen << plane1.nx, plane1.ny, plane1.nz, plane1.d;
  plane2_eigen << plane2.nx, plane2.ny, plane2.nz, plane2.d;

  correct_plane_direction(plane1_eigen);
  correct_plane_direction(plane2_eigen);

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

std_msgs::msg::ColorRGBA PlaneUtils::random_color() {
  std_msgs::msg::ColorRGBA color;
  color.r = rand() % 256;
  color.b = rand() % 256;
  color.g = rand() % 256;

  return color;
}

std::vector<double> PlaneUtils::random_color_vec() {
  std::vector<double> color;
  color.resize(3);
  color[0] = rand() % 256;
  color[1] = rand() % 256;
  color[2] = rand() % 256;

  return color;
}

std_msgs::msg::ColorRGBA PlaneUtils::rainbow_color_map(double h) {
  std_msgs::msg::ColorRGBA color;
  color.a = 255;
  // blend over HSV-values (more colors)

  // if (h == 0) {
  //   color.r = 0;
  //   color.g = 0;
  //   color.b = 0;
  //   return color;
  // }

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = 255 * v;
      color.g = 255 * n;
      color.b = 255 * m;
      break;
    case 1:
      color.r = 255 * n;
      color.g = 255 * v;
      color.b = 255 * m;
      break;
    case 2:
      color.r = 255 * m;
      color.g = 255 * v;
      color.b = 255 * n;
      break;
    case 3:
      color.r = 255 * m;
      color.g = 255 * n;
      color.b = 255 * v;
      break;
    case 4:
      color.r = 255 * n;
      color.g = 255 * m;
      color.b = 255 * v;
      break;
    case 5:
      color.r = 255 * v;
      color.g = 255 * m;
      color.b = 255 * n;
      break;
    default:
      color.r = 255;
      color.g = 127;
      color.b = 127;
      break;
  }
  return color;
}

std::vector<double> PlaneUtils::get_predefined_colors(const int id) {
  switch (id) {
    case 0:
      return {255, 0, 0};  // Red
    case 1:
      return {0, 255, 0};  // Green
    case 2:
      return {255, 165, 0};  // Orange
    case 3:
      return {128, 128, 0};  // Olive
    case 4:
      return {128, 64, 0};  // Brownish shade
    default:
      return PlaneUtils::random_color_vec();
  }
}

}  // namespace s_graphs
