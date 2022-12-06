// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_MAPPER_HPP
#define FLOOR_MAPPER_HPP

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
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/plane_utils.hpp>

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
class FloorMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  /**
   * @brief Constructor for the class FloorMapper
   *
   * @param private_nh
   */
  FloorMapper(const ros::NodeHandle& private_nh);
  ~FloorMapper();

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param floors_vec
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void lookup_floors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms, const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms);

private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param floors_vec
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void factor_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms, const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_node
   * @param room_data
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void update_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node, const s_graphs::RoomData room_data, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms, const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms);

private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_pose
   * @param floor_node
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  void factor_floor_room_nodes(std::unique_ptr<GraphSLAM>& graph_slam, const Eigen::Vector2d& floor_pose, g2o::VertexRoomXYLB* floor_node, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms, const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param graph_slam
   * @param floor_node
   */
  void remove_floor_room_nodes(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node);
};

}  // namespace s_graphs

#endif  // FLOOR_MAPPER_HPP
