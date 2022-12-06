// SPDX-License-Identifier: BSD-2-Clause

#ifndef NEIGHBOUR_MAPPER_HPP
#define NEIGHBOUR_MAPPER_HPP

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
#include <s_graphs/infinite_rooms.hpp>
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
#include <g2o/edge_plane.hpp>
#include <g2o/edge_corridor_plane.hpp>
#include <g2o/edge_room.hpp>

namespace s_graphs {

/**
 * @brief
 */
class NeighbourMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  /**
   * @brief Contructor of class NeighbourMapper
   *
   * @param private_nh
   */
  NeighbourMapper(const ros::NodeHandle& private_nh);
  ~NeighbourMapper();

public:
  /**
   * @brief Detect all the room neighbours from all room data
   *
   * @param graph_slam
   * @param room_msg
   * @param x_corridors
   * @param y_corridors
   * @param rooms_vec
   */
  void detect_room_neighbours(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomsData& room_msg, std::vector<InfiniteRooms>& x_corridors, std::vector<InfiniteRooms>& y_corridors, std::vector<Rooms>& rooms_vec);

  /**
   * @brief Factor the room neighbours between two rooms/corridors
   *
   * @param graph_slam
   * @param room_msg
   * @param x_corridors
   * @param y_corridors
   * @param rooms_vec
   */
  void factor_room_neighbours(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomsData& room_msg, std::vector<InfiniteRooms>& x_corridors, std::vector<InfiniteRooms>& y_corridors, std::vector<Rooms>& rooms_vec);

private:
  /**
   * @brief Get the current pose between the two rooms
   *
   * @param room_msg_1
   * @param room_msg_2
   * @return
   */
  Eigen::Vector2d room_room_measurement(const s_graphs::RoomData& room_msg_1, const s_graphs::RoomData& room_msg_2);

  /**
   * @brief Get the current pose between the room and x_corridor
   *
   * @param room_msg
   * @param x_corridor_msg
   * @return
   */
  double room_x_corridor_measurement(const s_graphs::RoomData& room_msg, const s_graphs::RoomData& x_corridor_msg);

  /**
   * @brief Get the current pose between the room and y_corridor
   *
   * @param room_msg
   * @param y_corridor_msg
   * @return
   */
  double room_y_corridor_measurement(const s_graphs::RoomData& room_msg, const s_graphs::RoomData& y_corridor_msg);

  /**
   * @brief
   *
   * @param x_corridor_msg1
   * @param x_corridor_msg2
   * @return
   */
  double x_corridor_x_corridor_measurement(const s_graphs::RoomData& x_corridor_msg1, const s_graphs::RoomData& x_corridor_msg2);

  /**
   * @brief
   *
   * @param y_corridor_msg1
   * @param y_corridor_msg2
   * @return
   */
  double y_corridor_y_corridor_measurement(const s_graphs::RoomData& y_corridor_msg1, const s_graphs::RoomData& y_corridor_msg2);

  /**
   * @brief Factor edges between neighbouring rooms
   *
   * @param graph_slam
   * @param room1
   * @param room2
   * @param room_room_meas
   */
  void factor_room_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room1, const s_graphs::Rooms& room2, Eigen::Vector2d room_room_meas);

  /**
   * @brief factor edges between room x_corridor
   *
   * @param graph_slam
   * @param room
   * @param x_corridor
   * @param room_x_corr_meas
   */
  void factor_room_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room, const s_graphs::InfiniteRooms& x_corridor, double room_x_corr_meas);

  /**
   * @brief factor edges between room and y corridor
   *
   * @param graph_slam
   * @param room
   * @param y_corridor
   * @param room_y_corr_meas
   */
  void factor_room_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room, const s_graphs::InfiniteRooms& y_corridor, double room_y_corr_meas);

  /**
   * @brief
   *
   * @param graph_slam
   * @param x_corridor
   * @param room
   * @param x_corr_room_meas
   */
  void factor_x_corridor_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& x_corridor, const s_graphs::Rooms& room, double x_corr_room_meas);

  /**
   * @brief
   *
   * @param graph_slam
   * @param x_corridor1
   * @param x_corridor2
   * @param x_corr_x_corr_meas
   */
  void factor_x_corridor_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& x_corridor1, const s_graphs::InfiniteRooms& x_corridor2, double x_corr_x_corr_meas);

  /**
   * @brief
   *
   * @param graph_slam
   * @param x_corridor
   * @param y_corridor
   */
  void factor_x_corridor_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& x_corridor, const s_graphs::InfiniteRooms& y_corridor);

  /**
   * @brief
   *
   * @param graph_slam
   * @param y_corridor
   * @param room
   * @param y_corr_room_meas
   */
  void factor_y_corridor_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& y_corridor, const s_graphs::Rooms& room, double y_corr_room_meas);

  /**
   * @brief
   *
   * @param graph_slam
   * @param y_corridor
   * @param x_corridor
   */
  void factor_y_corridor_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& y_corridor, const s_graphs::InfiniteRooms& x_corridor);

  /**
   * @brief
   *
   * @param graph_slam
   * @param y_corridor1
   * @param y_corridor2
   * @param y_corr_y_corr_meas
   */
  void factor_y_corridor_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::InfiniteRooms& y_corridor1, const s_graphs::InfiniteRooms& y_corridor2, double y_corr_y_corr_meas);
};

}  // namespace s_graphs

#endif  // NEIGHBOUR_MAPPER_HPP
