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
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>

#include <g2o/vertex_room.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_room.hpp>

namespace s_graphs {

/**
 * @brief
 */
class PlaneMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  /**
   * @brief Contructor of class PlaneMapper
   *
   * @param private_nh
   */
  PlaneMapper(const ros::NodeHandle& private_nh);
  ~PlaneMapper();

public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param keyframe
   * @param extracted_cloud_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void map_extracted_planes(std::unique_ptr<GraphSLAM>& graph_slam, KeyFrame::Ptr keyframe, const std::vector<pcl::PointCloud<PointNormal>::Ptr>& extracted_cloud_vec, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param keyframe
   * @param det_plane_body_frame
   * @return
   */
  g2o::Plane3D convert_plane_to_map_frame(const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_body_frame);

private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param keyframe
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int add_planes_to_graph(std::unique_ptr<GraphSLAM>& graph_slam, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_body_frame, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int sort_planes(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, const g2o::Plane3D& det_plane_body_frame, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int factor_planes(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, const g2o::Plane3D& det_plane_body_frame, std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param plane_type
   * @param keyframe
   * @param det_plane
   * @param cloud_seg_body
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  std::pair<int, int> associate_plane(const int& plane_type, const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane, const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void convert_plane_points_to_map(std::vector<VerticalPlanes>& x_vert_planes, std::vector<VerticalPlanes>& y_vert_planes, std::vector<HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_id
   * @param keyframe
   * @param det_plane_map_frame
   * @param found_infinite_room
   * @param found_room
   * @param plane_id_pair
   */
  void retrieve_plane_properties(const int& plane_type, const int& plane_id, const KeyFrame::Ptr& keyframe, const g2o::Plane3D& det_plane_map_frame, bool& found_infinite_room, bool& found_room, plane_data_list& plane_id_pair);

private:
  bool use_point_to_plane;
  double plane_information;
  double plane_dist_threshold;
  double plane_points_dist;
  double infinite_room_min_plane_length;
  double room_min_plane_length, room_max_plane_length;
  double min_plane_points;
  bool use_infinite_room_constraint;
  bool use_room_constraint;

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;
};

}  // namespace s_graphs

#endif  // PLANE_MAPPER_HPP
