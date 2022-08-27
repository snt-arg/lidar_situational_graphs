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

class MapperUtils {
public:
  MapperUtils() {
    plane_utils.reset(new PlaneUtils());
  }

public:
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  inline float point_difference(int plane_type, pcl::PointXY p1, pcl::PointXY p2) {
    float point_diff = 0;

    if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      p1.x = 0;
      p2.x = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }
    if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      p1.y = 0;
      p2.y = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }

    return point_diff;
  }

  /**
   * @brief this method add parallel constraint between the planes of rooms or corridors
   */
  void parallel_plane_constraint(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_parallel_edge(plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  /**
   * @brief this method adds perpendicular constraint between the planes of rooms or corridors
   */
  void perpendicular_plane_constraint(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_perpendicular_edge(plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }
};

/**
 * @brief this class provides tools for different analysis over open space clusters to generate rooms
 */
class InfiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  InfiniteRoomMapper(const ros::NodeHandle& private_nh);
  ~InfiniteRoomMapper();

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  void lookup_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const std::vector<plane_data_list>& x_det_corridor_candidates, const std::vector<plane_data_list>& y_det_corridor_candidates, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);
  void lookup_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, const s_graphs::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors, const std::vector<Rooms>& rooms_vec);

  double corridor_measurement(int plane_type, double corr, const Eigen::Vector4d& plane);

private:
  /**
   * @brief sort corridors and add their possible candidates for refinement
   */
  std::vector<structure_data_list> sort_corridors(const int plane_type, const std::vector<plane_data_list>& corridor_candidates);

  std::vector<plane_data_list> refine_corridors(const std::vector<structure_data_list>& corr_vec);

  /**
   * @brief this method creates the corridor vertex and adds edges between the vertex the detected planes
   */
  void factor_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const int plane_type, const plane_data_list& corr_plane1_pair, const plane_data_list& corr_plane2_pair, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);

  std::pair<int, int> associate_corridors(const int& plane_type, const Eigen::Vector2d& corr_pose, const std::vector<Corridors>& x_corridors, const std::vector<Corridors>& y_corridors);
  std::pair<int, int> associate_corridors(const int& plane_type, const Eigen::Vector2d& corr_pose, const VerticalPlanes& plane1, const VerticalPlanes& plane2, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, const std::vector<Corridors>& x_corridors, const std::vector<Corridors>& y_corridors);

  bool check_corridor_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexCorridor* corr_node);

  /**
   * @brief map a new corridor from mapped room planes
   *
   */
  void map_corridor_from_existing_room(std::unique_ptr<GraphSLAM>& graph_slam, const int& plane_type, const s_graphs::RoomData& det_room_data, const s_graphs::Rooms& matched_room, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors);

private:
  void parallel_plane_constraint(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node);

private:
  double corridor_plane_length_diff_threshold, corridor_point_diff_threshold;
  double corridor_min_width, corridor_max_width;
  double corridor_information;
  double corridor_dist_threshold;
  double corridor_min_seg_dist;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
};

class FiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  FiniteRoomMapper(const ros::NodeHandle& private_nh);
  ~FiniteRoomMapper();

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  void lookup_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const std::vector<plane_data_list>& x_det_room_candidates, const std::vector<plane_data_list>& y_det_room_candidates, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Rooms>& rooms_vec);
  void lookup_rooms(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, const std::vector<Corridors>& x_corridors, const std::vector<Corridors>& y_corridors, std::vector<Rooms>& rooms_vec);
  Eigen::Vector2d room_measurement(const int& plane_type, const Eigen::Vector2d& room_pose, const Eigen::Vector4d& plane);

private:
  /**
   * @brief sort the rooms candidates
   */
  std::vector<structure_data_list> sort_rooms(const int& plane_type, const std::vector<plane_data_list>& room_candidates);

  /**
   * @brief refine the sorted room candidates
   */
  std::pair<std::vector<plane_data_list>, std::vector<plane_data_list>> refine_rooms(std::vector<structure_data_list> x_room_vec, std::vector<structure_data_list> y_room_vec);

  /**
   * @brief this method creates the room vertex and adds edges between the vertex and detected planes
   */
  void factor_rooms(std::unique_ptr<GraphSLAM>& graph_slam, std::vector<plane_data_list> x_room_pair_vec, std::vector<plane_data_list> y_room_pair_vec, const std::vector<VerticalPlanes>& x_vert_planes, const std::vector<VerticalPlanes>& y_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes, std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes, std::vector<Rooms>& rooms_vec);
  Eigen::Vector2d compute_room_pose(const std::vector<plane_data_list>& x_room_pair_vec, const std::vector<plane_data_list>& y_room_pair_vec);
  std::pair<int, int> associate_rooms(const Eigen::Vector2d& room_pose, const std::vector<Rooms>& rooms_vec);
  bool check_room_ids(const int plane_type, const std::set<g2o::HyperGraph::Edge*>& plane_edges, const g2o::VertexRoomXYLB* room_node);

  /**
   * @brief map a new room from mapped corridor planes
   *
   */
  void map_room_from_existing_corridors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_x_corridor, const s_graphs::Corridors& matched_y_corridor, std::vector<Rooms>& rooms_vec);

  /**
   * @brief map a new room from mapped x corridor planes
   *
   */
  void map_room_from_existing_x_corridor(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_x_corridor, std::vector<Rooms>& rooms_vec);

  /**
   * @brief map a new room from mapped y corridor planes
   *
   */
  void map_room_from_existing_y_corridor(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_y_corridor, std::vector<Rooms>& rooms_vec);

private:
  double room_width_diff_threshold;
  double room_plane_length_diff_threshold, room_point_diff_threshold;
  double room_min_width, room_max_width;
  double room_information;
  double room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
};

}  // namespace s_graphs

#endif  // ROOM_MAPPER_HPP
