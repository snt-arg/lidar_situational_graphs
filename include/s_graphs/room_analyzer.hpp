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
#include <pcl/filters/voxel_grid.h>

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

  /**
   * @brief
   *
   * @param
   * @return
   */
  void analyze_skeleton_graph(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg);

  /**
   * @brief
   *
   * @param
   * @return
   */
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> get_cloud_clusters() {
    return cloud_clusters;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  std::vector<std::pair<int, int>> get_connected_graph() {
    return connected_subgraphs;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  visualization_msgs::MarkerArray get_makerarray_clusters() {
    return connected_clusters_marker_array;
  }

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_room_planes(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, pcl::PointXY p_min, pcl::PointXY p_max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2, bool& found_x1_plane, bool& found_x2_plane, bool& found_y1_plane, bool& found_y2_plane);

  /**
   * @brief
   *
   * @param
   * @return
   */
  void get_convex_hull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& area);

  /**
   * @brief
   *
   * @param
   * @return
   */
  void get_cluster_endpoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, pcl::PointXY& p1, pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const pcl::PointXY& p1, const pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const geometry_msgs::Point& room_center);

  /**
   * @brief
   *
   * @param
   * @return
   */
  geometry_msgs::Point get_room_length(const pcl::PointXY& p1, const pcl::PointXY& p2);

  /**
   * @brief
   *
   * @param
   * @return
   */
  geometry_msgs::Point get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2, Eigen::Vector2d& cluster_center);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool perform_room_segmentation(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, int& room_cluster_counter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, std::vector<s_graphs::RoomData>& room_candidates_vec, std::vector<std::pair<int, int>> connected_subgraph_map);

  /**
   * @brief
   *
   * @param
   * @return
   */
  void downsample_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull);

private:
  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_x1yplane_alignment(const std::vector<geometry_msgs::Vector3> x_plane1_points, const std::vector<geometry_msgs::Vector3> y_plane_points);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_x2yplane_alignment(const std::vector<geometry_msgs::Vector3> x_plane2_points, const std::vector<geometry_msgs::Vector3> y_plane_point);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_y1xplane_alignment(const std::vector<geometry_msgs::Vector3> y_plane1_points, const std::vector<geometry_msgs::Vector3> x_plane_points);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool check_y2xplane_alignment(const std::vector<geometry_msgs::Vector3> y_plane2_points, const std::vector<geometry_msgs::Vector3> x_plane_points);

private:
  ros::NodeHandle nh;
  std::shared_ptr<PlaneUtils> plane_utils;
  int vertex_neigh_thres;

  std::mutex skeleton_graph_mutex;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
  std::vector<std::pair<int, int>> connected_subgraphs;
  visualization_msgs::MarkerArray connected_clusters_marker_array;

private:
  /**
   * @brief
   *
   * @param
   * @return
   */
  std::vector<float> find_plane_points(const pcl::PointXY& start_point, const pcl::PointXY& end_point, const s_graphs::PlaneData& plane);

  /**
   * @brief
   *
   * @param
   * @return
   */
  int find_plane_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const s_graphs::PlaneData& plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sub_cloud_hull);

  /**
   * @brief
   *
   * @param
   * @return
   */
  pcl::PointXYZRGB compute_centroid(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2);
};
}  // namespace s_graphs

#endif  // ROOM_ANALYZER_HPP
