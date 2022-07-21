// SPDX-License-Identifier: BSD-2-Clause

#ifndef PLANE_MAPPER_HPP
#define PLANE_MAPPER_HPP

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
#include <pcl/common/common.h>

#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/corridors.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>

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

class PlaneMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  PlaneMapper(const ros::NodeHandle& private_nh);
  ~PlaneMapper();

public:
  int map_detected_planes(std::unique_ptr<GraphSLAM>& graph_slam, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_body_frame, bool& found_corridor, bool& found_room, plane_data_list& plane_id_pair, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

private:
  g2o::Plane3D plane_in_map_frame(const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_body_frame);
  int sort_planes(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, const g2o::Plane3D& det_plane_body_frame, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);
  int factor_planes(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, const g2o::Plane3D& det_plane_body_frame, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);
  std::pair<int, int> associate_plane(const int& plane_type, const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane, const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const std::vector<HorizontalPlanes>& hort_planes);
  void convert_plane_points_to_map(std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);
  void get_plane_properties(const int& plane_type, const int& plane_id, const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, bool& found_corridor, bool& found_room, plane_data_list& plane_id_pair);

private:
  bool use_point_to_plane;
  double plane_dist_threshold;
  double plane_points_dist;
  double corridor_min_plane_length;
  double room_min_plane_length, room_max_plane_length;

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;
};

}  // namespace s_graphs

#endif  // PLANE_MAPPER_HPP
