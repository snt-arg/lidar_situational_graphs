// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROOM_MAPPER_HPP
#define ROOM_MAPPER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>

#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/corridors.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/plane_utils.hpp>

#include <g2o/vertex_room.hpp>
#include <g2o/vertex_corridor.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/edge_corridor_plane.hpp>
#include <g2o/edge_room.hpp>

namespace s_graphs {

struct plane_data_list {
  plane_data_list() : plane_centroid(0, 0, 0), connected_id(-1) {
    connected_neighbour_ids.clear();
  }
  // g2o::Plane3D plane;
  g2o::Plane3D plane_unflipped;
  int plane_id;
  int connected_id;
  std::vector<int> connected_neighbour_ids;
  pcl::PointXY start_point, end_point;
  float plane_length;
  g2o::VertexSE3* keyframe_node;
  Eigen::Vector3d plane_centroid;
};

struct structure_data_list {
  plane_data_list plane1;
  plane_data_list plane2;
  float width;
  float length_diff;
  float avg_point_diff;
};

/**
 * @brief this class provides tools for different analysis over open space clusters to generate rooms
 */
class RoomMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  RoomMapper(const ros::NodeHandle& private_nh);
  ~RoomMapper();

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  void lookup_corridors(std::unique_ptr<GraphSLAM>& graph_slam, std::vector<plane_data_list> x_det_corridor_candidates, std::vector<plane_data_list> y_det_corridor_candidates, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);

private:
  std::vector<structure_data_list> sort_corridors(int plane_type, std::vector<plane_data_list> corridor_candidates);
  std::vector<plane_data_list> refine_corridors(std::vector<structure_data_list> corr_vec);

  void factor_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const int plane_type, const plane_data_list& corr_plane1_pair, const plane_data_list& corr_plane2_pair, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);
  Eigen::Vector2d compute_corridor_pose(int plane_type, Eigen::Vector3d keyframe_pose, Eigen::Vector4d v1, Eigen::Vector4d v2);
  std::pair<int, int> associate_corridors(int plane_type, Eigen::Vector2d corr_pose, const std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);
  double corridor_measurement(int plane_type, double corr, Eigen::Vector4d plane);

private:
  void parallel_plane_constraint(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node);

private:
  float width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2);
  float point_difference(int plane_type, pcl::PointXY p1, pcl::PointXY p2);

private:
  double corridor_plane_length_diff_threshold, corridor_point_diff_threshold;
  double corridor_min_width, corridor_max_width;
  double corridor_information;
  double corridor_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
};
}  // namespace s_graphs

#endif  // ROOM_MAPPER_HPP
