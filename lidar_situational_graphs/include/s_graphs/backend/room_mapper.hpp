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

#ifndef ROOM_MAPPER_HPP
#define ROOM_MAPPER_HPP

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

/**
 * @brief
 */
class MapperUtils {
 public:
  /**
   * @brief Constructor of the class MapperUtils
   *
   * @param
   * @return
   */
  MapperUtils() {}

 public:
  /**
   * @brief
   *
   * @param plane_type
   * @param p1
   * @param p2
   * @return
   */
  static inline float point_difference(int plane_type,
                                       pcl::PointXY p1,
                                       pcl::PointXY p2) {
    float point_diff = 0;

    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      p1.x = 0;
      p2.x = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }
    if (plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
      p1.y = 0;
      p2.y = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }

    return point_diff;
  }

  /**
   * @brief This method add parallel constraint between the planes of rooms or
   * infinite_rooms
   *
   * @param graph_slam
   * @param plane1_node
   * @param plane2_node
   */
  static void parallel_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
                                        g2o::VertexPlane* plane1_node,
                                        g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_parallel_edge(
        plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  /**
   * @brief This method adds perpendicular constraint between the planes of
   * rooms or infinite_rooms
   *
   * @param graph_slam
   * @pram plane1_node
   * @pram plane2_node
   */
  static void perpendicular_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
                                             g2o::VertexPlane* plane1_node,
                                             g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_perpendicular_edge(
        plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  static bool check_plane_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                              const g2o::VertexPlane* plane_node) {
    for (auto edge_itr = plane_edges.begin(); edge_itr != plane_edges.end();
         ++edge_itr) {
      g2o::Edge2Planes* edge_2planes = dynamic_cast<g2o::Edge2Planes*>(*edge_itr);
      if (edge_2planes) {
        g2o::VertexPlane* found_plane1_node =
            dynamic_cast<g2o::VertexPlane*>(edge_2planes->vertices()[0]);
        g2o::VertexPlane* found_plane2_node =
            dynamic_cast<g2o::VertexPlane*>(edge_2planes->vertices()[1]);

        if (found_plane1_node->id() == plane_node->id() ||
            found_plane2_node->id() == plane_node->id())
          return true;
      }
    }

    return false;
  }
};

/**
 * @brief Class that provides tools for different analysis over open space
 * clusters to generate rooms
 */
class InfiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor of the class InfiniteRoomMapper
   *
   * @param private_nh
   * @return
   */
  InfiniteRoomMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex);
  ~InfiniteRoomMapper();

 private:
  rclcpp::Node::SharedPtr node_obj;

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param plane_type
   * @param room_data
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param rooms_vec
   * @return bool
   */
  bool lookup_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const int& plane_type,
      const situational_graphs_msgs::msg::RoomData room_data,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      const std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id);

 private:
  /**
   * @brief Creates the infinite_room vertex and adds edges between the vertex
   * the detected planes
   *
   * @param graph_slam
   * @param plane_type
   * @param corr_plane1_pair
   * @param corr_plane2_pair
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  bool factor_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const int plane_type,
      const plane_data_list& corr_plane1_pair,
      const plane_data_list& corr_plane2_pair,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      const Eigen::Isometry3d& room_center,
      const Eigen::Isometry3d& cluster_center,
      int& room_id);

  /**
   * @brief
   *
   * @param plane_type
   * @param corr_pose
   * @param plane1
   * @param plane2
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param detected_mapped_plane_pairs
   * @return
   */
  int associate_infinite_rooms(
      const int& plane_type,
      const Eigen::Isometry3d& room_center,
      const VerticalPlanes& plane1,
      const VerticalPlanes& plane2,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
          detected_mapped_plane_pairs);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_edges
   * @param corr_node
   * @return Success or failure
   */
  bool check_infinite_room_ids(const int plane_type,
                               const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                               const g2o::VertexRoom* corr_node);

 private:
  /**
   * @brief
   *
   * @param graph_slam
   * @param plane1_node
   * @param plane2_node
   */
  void parallel_plane_constraint(std::shared_ptr<GraphSLAM>& graph_slam,
                                 g2o::VertexPlane* plane1_node,
                                 g2o::VertexPlane* plane2_node);

  /**
   * @brief
   *
   * @param room_data_association
   * @param room_plane1_pair
   * @param room_plane2_pair
   * @param infinite_rooms
   * @return true
   * @return false
   */
  bool insert_infinite_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      int room_data_association,
      int& room_id,
      const Eigen::Isometry3d& room_center,
      const Eigen::Isometry3d& cluster_center,
      const VerticalPlanes& plane1,
      const VerticalPlanes& plane2,
      const plane_data_list& room_plane1_pair,
      const plane_data_list& room_plane2_pair,
      std::unordered_map<int, InfiniteRooms>& infinite_rooms,
      const std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
          detected_mapped_plane_pairs);

  /**
   * @brief
   *
   * @param plane1
   * @param plane2
   * @param infinite_room
   * @param vert_planes
   * @return std::vector<std::pair<VerticalPlanes, VerticalPlanes>>
   */
  std::vector<std::pair<VerticalPlanes, VerticalPlanes>> match_planes(
      bool& plane1_min_segment,
      bool& plane2_min_segment,
      const VerticalPlanes& plane1,
      const VerticalPlanes& plane2,
      const InfiniteRooms& infinite_room,
      const std::unordered_map<int, VerticalPlanes>& vert_planes);

 private:
  double infinite_room_information;
  double infinite_room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  double dupl_plane_matching_information;
  std::mutex& shared_graph_mutex;
};

/**
 * @brief
 */
class FiniteRoomMapper : public MapperUtils {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor of class FiniteRoomMapper.
   *
   * @param private_nh
   */
  FiniteRoomMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex);
  ~FiniteRoomMapper();

 private:
  rclcpp::Node::SharedPtr node_obj;

 public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param room_data
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param rooms_vec
   */
  bool lookup_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData room_data,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id);

  /**
   * @brief
   *
   * @param plane_type
   * @param room_pose
   * @param plane
   * @return
   */
  double room_measurement(const int& plane_type,
                          const Eigen::Vector2d& room_pose,
                          const Eigen::Vector4d& plane);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param x_vert_planes
   * @param y_vert_planes
   * @param room
   */
  void factor_saved_rooms(const std::shared_ptr<GraphSLAM> covisibility_graph,
                          const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                          const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                          const Rooms& room);

 private:
  /**
   * @brief Creates the room vertex and adds edges between the vertex and
   * detected planes
   *
   * @param graph_slam
   * @param x_room_pair_vec
   * @param y_room_pair_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   * @param rooms_vec
   * @param cluster_array
   */
  bool factor_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      std::vector<plane_data_list> x_room_pair_vec,
      std::vector<plane_data_list> y_room_pair_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>>& dupl_y_vert_planes,
      std::unordered_map<int, Rooms>& rooms_vec,
      int& room_id,
      const Eigen::Isometry3d& room_center,
      const visualization_msgs::msg::MarkerArray& cluster_array);

  /**
   * @brief
   *
   * @param room_pose
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   * @param detected_mapped_plane_pairs
   * @return
   */
  int associate_rooms(const Eigen::Isometry3d& room_center,
                      const std::unordered_map<int, Rooms>& rooms_vec,
                      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                      const VerticalPlanes& x_plane1,
                      const VerticalPlanes& x_plane2,
                      const VerticalPlanes& y_plane1,
                      const VerticalPlanes& y_plane2,
                      std::vector<std::pair<VerticalPlanes, VerticalPlanes>>&
                          detected_mapped_plane_pairs);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_edges
   * @param room_node
   * @return Success or failure
   */
  bool check_room_ids(const int plane_type,
                      const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                      const g2o::VertexRoom* room_node);
  bool check_plane_ids(const std::set<g2o::HyperGraph::Edge*>& plane_edges,
                       const g2o::VertexPlane* plane_node);
  /**
   * @brief Map a new room from mapped infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_x_infinite_room
   * @param matched_y_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_infinite_rooms(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_x_infinite_room,
      const s_graphs::InfiniteRooms& matched_y_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief map a new room from mapped x infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_x_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_x_infinite_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_x_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief map a new room from mapped y infinite_room planes
   *
   * @param graph_slam
   * @param det_room_data
   * @param matched_y_infinite_room
   * @param rooms_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param x_plane1
   * @param x_plane2
   * @param y_plane1
   * @param y_plane2
   */
  void map_room_from_existing_y_infinite_room(
      std::shared_ptr<GraphSLAM>& graph_slam,
      const situational_graphs_msgs::msg::RoomData& det_room_data,
      const s_graphs::InfiniteRooms& matched_y_infinite_room,
      const Eigen::Isometry3d& room_center,
      std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const VerticalPlanes& x_plane1,
      const VerticalPlanes& x_plane2,
      const VerticalPlanes& y_plane1,
      const VerticalPlanes& y_plane2);

  /**
   * @brief remove the infinite_room overlapped by a room
   *
   */
  void remove_mapped_infinite_room(
      const int plane_type,
      std::shared_ptr<GraphSLAM>& graph_slam,
      s_graphs::InfiniteRooms matched_infinite_room,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms);

 private:
  double room_information;
  double room_dist_threshold;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  double dupl_plane_matching_information;
  std::mutex& shared_graph_mutex;
};

}  // namespace s_graphs

#endif  // ROOM_MAPPER_HPP
