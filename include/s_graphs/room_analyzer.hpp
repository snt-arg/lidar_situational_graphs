// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROOM_ANALYZER_HPP
#define ROOM_ANALYZER_HPP

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
#include <s_graphs/plane_utils.hpp>

namespace s_graphs {

/**
 * @brief this class provides tools for different analysis over open space clusters to generate rooms
 */
class RoomAnalyzer {
public:
  RoomAnalyzer(const ros::NodeHandle& private_nh, std::shared_ptr<PlaneUtils> plane_utils_ptr);
  ~RoomAnalyzer();

  void analyze_skeleton_graph(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> get_cloud_clusters();
  std::vector<std::pair<int, int>> get_connected_graph();
  void get_room_planes(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, pcl::PointXY p_min, pcl::PointXY p_max, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2, bool& found_x1_plane, bool& found_x2_plane, bool& found_y1_plane, bool& found_y2_plane);
  void get_convex_hull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& area);
  void get_cluster_endpoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, pcl::PointXY& p1, pcl::PointXY& p2);
  bool get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const pcl::PointXY& p1, const pcl::PointXY& p2);
  bool get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const geometry_msgs::Point& room_center);

  geometry_msgs::Point get_room_length(pcl::PointXY p1, pcl::PointXY p2);
  geometry_msgs::Point get_room_center(pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData x_plane1, s_graphs::PlaneData x_plane2, s_graphs::PlaneData y_plane1, s_graphs::PlaneData y_plane2);
  geometry_msgs::Point get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2);
  void perform_room_segmentation(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, int& room_cluster_counter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, std::vector<s_graphs::RoomData>& room_candidates_vec, std::vector<std::pair<int, int>> connected_subgraph_map);

private:
  ros::NodeHandle nh;
  std::shared_ptr<PlaneUtils> plane_utils;

  std::mutex skeleton_graph_mutex;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters_;
  std::vector<std::pair<int, int>> connected_subgraphs_;

private:
  bool compute_diagonal_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const pcl::PointXY min, const pcl::PointXY max, pcl::PointXY& top_right, pcl::PointXY& bottom_right, pcl::PointXY& top_left, pcl::PointXY& bottom_left);
  std::vector<float> find_plane_points(const pcl::PointXY& start_point, const pcl::PointXY& end_point, const s_graphs::PlaneData& plane);
  int find_plane_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const s_graphs::PlaneData& plane);
  pcl::PointXYZRGB compute_centroid(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2);
  bool compute_point_difference(const double plane1_point, const double plane2_point);

  enum plane_class : uint8_t {
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
    HORT_PLANE = 2,
  };
};
}  // namespace s_graphs

#endif  // ROOM_ANALYZER_HPP
