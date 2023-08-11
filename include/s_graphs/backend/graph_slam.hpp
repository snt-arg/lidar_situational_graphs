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

// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>

#include <g2o/edge_doorway_two_rooms.hpp>
#include <g2o/edge_se3_two_planes.hpp>
#include <g2o/edge_se3_two_rooms.hpp>
#include <g2o/edge_wall_two_planes.hpp>
#include <g2o/vertex_deviation.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"
namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class VertexInfiniteRoom;
class VertexDeviation;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointToPlane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class EdgePlane;
class EdgePlaneIdentity;
class EdgePlaneParallel;
class EdgeSE3InfiniteRoom;
class EdgeInfiniteRoomXPlane;
class EdgeInfiniteRoomYPlane;
class EdgeSE3Room;
class EdgeRoom2Planes;
class EdgeRoom4Planes;
class EdgeRoomRoom;
class EdgeFloorRoom;
class EdgeXInfiniteRoomXInfiniteRoom;
class EdgeYInfiniteRoomYInfiniteRoom;
class EdgePlanePerpendicular;
class Edge2Planes;
class EdgeEdgeSE3PlanePlane;
class EdgePlanePriorNormal;
class EdgePlanePriorDistance;
class EdgeSE3RoomRoom;
class Edge2Rooms;
class RobustKernelFactory;
class VertexRoom;
class VertexFloor;
class VertexDoorWay;
class VertexWallXYZ;

}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 */
class GraphSLAM {
 public:
  /**
   * @brief Constructor for class GraphSLAM.
   *
   * @param solver_type Default value is lm_var
   */
  GraphSLAM(const std::string& solver_type = "lm_var_cholmod", bool save_time = false);
  virtual ~GraphSLAM();

  /**
   * @brief Counts the number of vertices in the graph.
   *
   * @return Number of vertices in the graph.
   */
  int retrieve_total_nbr_of_vertices() const;

  /**
   * @brief Counts the number of edges in the graph.
   *
   * @return Number of edges in the graph.
   */
  int retrive_total_nbr_of_edges() const;

  /**
   * @brief Counts the number of vertices in the graph that are local.
   *
   * @return Number of vertices in the graph that are local.
   */
  int retrieve_local_nbr_of_vertices() const;

  /**
   * @brief Counts the number of edges in the graph that are local.
   *
   * @return Number of edges in the graph that are local.
   */
  int retrieve_local_nbr_of_edges() const;

  /**
   * @brief
   *
   * @return
   */
  int increment_local_nbr_of_vertices();

  /**
   * @brief
   *
   * @return
   */
  int increment_local_nbr_of_edges();

  /**
   * @brief Set the current solver type
   *
   * @param solver_type
   */
  void select_solver_type(const std::string& solver_type);

  /**
   * @brief Add a SE3 node to the graph.
   *
   * @param pose
   * @return Registered node
   */
  g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

  /**
   * @brief copy an SE3 node from another graph.
   *
   * @param node
   * @return Registered node
   */
  g2o::VertexSE3* copy_se3_node(const g2o::VertexSE3* node);

  /**
   * @brief Add a plane node to the graph
   *
   * @param plane_coeffs
   * @return Registered node
   */
  g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

  /**
   * @brief Add a plane node to the graph
   *
   * @param plane_coeffs
   * @param id
   * @return Registered node
   */
  g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs, const int id);

  /**
   * @brief copy a plane node from another graph
   *
   * @param node
   * @return Registered node
   */
  g2o::VertexPlane* copy_plane_node(const g2o::VertexPlane* node);

  /**
   * @brief remove a plane node from the graph
   * @param plane vertex
   * @return success or failure
   */
  bool remove_plane_node(g2o::VertexPlane* plane_vertex);

  /**
   * @brief remove a room node from the graph
   * @param room vertex
   * @return success or failure
   */
  bool remove_room_node(g2o::VertexRoom* room_vertex);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return Registered node
   */
  g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz);

  /**
   * @brief Add a room node to the graph
   *
   * @param room_pose
   * @return Registered node
   */
  g2o::VertexRoom* add_room_node(const Eigen::Isometry3d& room_pose);

  /**
   * @brief Add a doorway node to the graph
   *
   * @param doorway_pose
   * @return Registered node
   */
  g2o::VertexDoorWay* add_doorway_node(const Eigen::Isometry3d& doorway_pose);

  /**
   * @brief copy a room node from another graph
   *
   * @param node
   * @return Registered node
   */
  g2o::VertexRoom* copy_room_node(const g2o::VertexRoom* node);

  /**
   * @brief Add a floor node to the graph
   *
   * @param floor_pose
   * @return Registered node
   */
  g2o::VertexFloor* add_floor_node(const Eigen::Isometry3d& floor_pose);

  /**
   * @brief copy a floor node from another
   *
   * @param node
   * @return Registered node
   */
  g2o::VertexFloor* copy_floor_node(const g2o::VertexFloor* node);

  /**
   * @brief Update the floor node estimate in the graph
   *
   * @param floor_node
   * @para floor_pose
   * @return Registered node
   */
  void update_floor_node(g2o::VertexFloor* floor_node,
                         const Eigen::Isometry3d& floor_pose);

  /**
   * @brief add a Wall node to the graph
   * @param wall_center
   * @return registered node
   */

  g2o::VertexWallXYZ* add_wall_node(const Eigen::Vector3d& wall_center);
  /**
   * @brief Add a SE3 Deviation node to the graph.
   *
   * @param pose
   * @return Registered node
   */

  g2o::VertexDeviation* add_deviation_node(const Eigen::Isometry3d& pose);
  /**
   * @brief Add an edge between SE3 nodes
   *
   * @param v1: node1
   * @param v2: node2
   * @param relative_pose: relative pose between node1 and node2
   * @param information_matrix: information matrix (it must be 6x6)
   * @return registered edge
   */

  g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1,
                             g2o::VertexSE3* v2,
                             const Eigen::Isometry3d& relative_pose,
                             const Eigen::MatrixXd& information_matrix);

  /**
   * @brief copy an edge from another graph
   *
   * @param e: edge
   * @return registered edge
   */
  g2o::EdgeSE3* copy_se3_edge(g2o::EdgeSE3* e, g2o::VertexSE3* v1, g2o::VertexSE3* v2);

  /**
   * @brief Add an edge between an SE3 node and a plane node
   *
   * @param v_se3: SE3 node
   * @param v_plane: plane node
   * @param plane_coeffs: plane coefficients w.r.t. v_se3
   * @param information_matrix: information matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3,
                                        g2o::VertexPlane* v_plane,
                                        const Eigen::Vector4d& plane_coeffs,
                                        const Eigen::MatrixXd& information_matrix);
  /**
   * @brief copy an edge from another graph
   *
   * @param e: SE3plane edge
   * @param v1: se3 node
   * @param v2: plane node
   * @return registered edge
   */
  g2o::EdgeSE3Plane* copy_se3_plane_edge(g2o::EdgeSE3Plane* e,
                                         g2o::VertexSE3* v1,
                                         g2o::VertexPlane* v2);

  /**
   * @brief Remove a plane edge from the graph
   *
   * @param se3_plane_edge
   * @return Succes or failure
   */
  bool remove_se3_plane_edge(g2o::EdgeSE3Plane* se3_plane_edge);

  /**
   * @brief Add an edge between an SE3 node and to a plane using point to plane
   * distances
   *
   * @param v_se3: SE3 node
   * @param v_plane: plane node
   * @param points_matrix:  plane coefficients w.r.t. v_se3
   * @param information_matrix: information matrix (it must be 1x1)
   * @return registered edge
   */
  g2o::EdgeSE3PointToPlane* add_se3_point_to_plane_edge(
      g2o::VertexSE3* v_se3,
      g2o::VertexPlane* v_plane,
      const Eigen::Matrix4d& points_matrix,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Add an edge between an SE3 node and a point_xyz node
   *
   * @param v_se3: SE3 node
   * @param v_xyz: point_xyz node
   * @param xyz: xyz coordinate
   * @param information:  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(
      g2o::VertexSE3* v_se3,
      g2o::VertexPointXYZ* v_xyz,
      const Eigen::Vector3d& xyz,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Add a prior edge to an SE3 node
   *
   * @param v: Plane
   * @param normal
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge(
      g2o::VertexPlane* v,
      const Eigen::Vector3d& normal,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v: Plane
   * @param distance
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge(
      g2o::VertexPlane* v,
      double distance,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: Node
   * @param xy
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3,
                                             const Eigen::Vector2d& xy,
                                             const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: Node
   * @param xyz
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(
      g2o::VertexSE3* v_se3,
      const Eigen::Vector3d& xyz,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: node
   * @param quat: Quarternion
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(
      g2o::VertexSE3* v_se3,
      const Eigen::Quaterniond& quat,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3
   * @param direction
   * @param measurement
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(
      g2o::VertexSE3* v_se3,
      const Eigen::Vector3d& direction,
      const Eigen::Vector3d& measurement,
      const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlane* add_plane_edge(g2o::VertexPlane* v_plane1,
                                 g2o::VertexPlane* v_plane2,
                                 const Eigen::Vector4d& measurement,
                                 const Eigen::Matrix4d& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlaneIdentity* add_plane_identity_edge(g2o::VertexPlane* v_plane1,
                                                  g2o::VertexPlane* v_plane2,
                                                  const Eigen::Vector4d& measurement,
                                                  const Eigen::Matrix4d& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlaneParallel* add_plane_parallel_edge(g2o::VertexPlane* v_plane1,
                                                  g2o::VertexPlane* v_plane2,
                                                  const Eigen::Vector3d& measurement,
                                                  const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge(
      g2o::VertexPlane* v_plane1,
      g2o::VertexPlane* v_plane2,
      const Eigen::Vector3d& measurement,
      const Eigen::MatrixXd& information);

  /**
   * @brief add edges between duplicate planes
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::Edge2Planes* add_2planes_edge(g2o::VertexPlane* v_plane1,
                                     g2o::VertexPlane* v_plane2,
                                     const Eigen::MatrixXd& information);

  /**
   * @brief copy the 2planes edges from another graph
   *
   * @param e: 2planes edge
   * @param v1: plane1 edge
   * @param v2: plane2 edge
   * @return registered edge
   */
  g2o::Edge2Planes* copy_2planes_edge(g2o::Edge2Planes* e,
                                      g2o::VertexPlane* v1,
                                      g2o::VertexPlane* v2);
  /**
   * @brief Deviation connection edge between two planes
   *
   * @param v_se3: Deviation vertex
   * @param v1: plane1 edge
   * @param v2: plane2 edge
   * @return registered edge
   */
  g2o::EdgeSE3PlanePlane* add_se3_point_to_2planes_edge(
      g2o::VertexDeviation* v_se3,
      g2o::VertexPlane* v_plane1,
      g2o::VertexPlane* v_plane2,
      const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_se3
   * @param v_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeSE3Room* add_se3_room_edge(g2o::VertexSE3* v_se3,
                                      g2o::VertexRoom* v_room,
                                      const Eigen::Vector2d& measurement,
                                      const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param v_plane1
   * @param v_plane2
   * @param v_cluster_center
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoom2Planes* add_room_2planes_edge(g2o::VertexRoom* v_room,
                                              g2o::VertexPlane* v_plane1,
                                              g2o::VertexPlane* v_plane2,
                                              g2o::VertexRoom* v_cluster_center,
                                              const Eigen::MatrixXd& information);

  bool remove_room_2planes_edge(g2o::EdgeRoom2Planes* room_plane_edge);

  /**
   * @brief
   *
   * @param edge: edgeroom2plane
   * @param v1: vertex room
   * @param v2: vertex plane1
   * @param v3: vertex plane2
   * @return registered edge
   */
  g2o::EdgeRoom2Planes* copy_room_2planes_edge(g2o::EdgeRoom2Planes* e,
                                               g2o::VertexRoom* v1,
                                               g2o::VertexPlane* v2,
                                               g2o::VertexPlane* v3,
                                               g2o::VertexRoom* v4);

  /**
   * @brief
   *
   * @param v_room
   * @param v_plane1
   * @param v_plane2
   * @param v_yplane1
   * @param v_yplane2
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoom4Planes* add_room_4planes_edge(g2o::VertexRoom* v_room,
                                              g2o::VertexPlane* v_xplane1,
                                              g2o::VertexPlane* v_xplane2,
                                              g2o::VertexPlane* v_yplane1,
                                              g2o::VertexPlane* v_yplane2,
                                              const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param edge: edgeroom2plane
   * @param v1: vertex room
   * @param v2: vertex xplane1
   * @param v3: vertex xplane2
   * @param v4: vertex yplane1
   * @param v5: vertex yplane2
   * @return registered edge
   */
  g2o::EdgeRoom4Planes* copy_room_4planes_edge(g2o::EdgeRoom4Planes* e,
                                               g2o::VertexRoom* v1,
                                               g2o::VertexPlane* v2,
                                               g2o::VertexPlane* v3,
                                               g2o::VertexPlane* v4,
                                               g2o::VertexPlane* v5);

  /**
   * @brief add edge between floor and its rooms
   *
   * @param v_floor
   * @param v_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeFloorRoom* add_floor_room_edge(g2o::VertexFloor* v_floor,
                                          g2o::VertexRoom* v_room,
                                          const Eigen::Vector2d& measurement,
                                          const Eigen::MatrixXd& information);

  g2o::EdgeDoorWay2Rooms* add_doorway_2rooms_edge(g2o::VertexDoorWay* v_door_r1,
                                                  g2o::VertexDoorWay* v_door_r2,
                                                  g2o::VertexRoom* v_room1,
                                                  g2o::VertexRoom* v_room2,
                                                  const Eigen::MatrixXd& information);

  g2o::EdgeRoomRoom* add_room_room_edge(g2o::VertexRoom* v1,
                                        g2o::VertexRoom* v2,
                                        const Eigen::MatrixXd& information);

  /**
   * @brief deviations betwen rooms edge
   *
   * @param v1: vertex deviation
   * @param v2: vertex room
   * @param v3: vertex room
   * @return registered edge
   */
  g2o::EdgeSE3RoomRoom* add_deviation_two_rooms_edge(
      g2o::VertexDeviation* v1,
      g2o::VertexRoom* v2,
      g2o::VertexRoom* v3,
      const Eigen::MatrixXd& information);

  g2o::Edge2Rooms* add_2rooms_edge(g2o::VertexRoom* v1,
                                   g2o::VertexRoom* v2,
                                   const Eigen::MatrixXd& information);
  /**
   * @brief copy the floor room edge from a graph
   *
   * @param e: edge floor room
   * @param v1: vertex floor
   * @param v2: vertex room
   * @return registered edge
   */
  g2o::EdgeFloorRoom* copy_floor_room_edge(g2o::EdgeFloorRoom* e,
                                           g2o::VertexFloor* v1,
                                           g2o::VertexRoom* v2);

  /**
   * @brief
   *
   * @param room_room_edge
   * @return Succes or failure
   */
  bool remove_room_room_edge(g2o::EdgeFloorRoom* room_room_edge);

  /**
   * @brief
   *
   * @param v_xcorr1
   * @param v_xcorr2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeXInfiniteRoomXInfiniteRoom* add_x_infinite_room_x_infinite_room_edge(
      g2o::VertexInfiniteRoom* v_xcorr1,
      g2o::VertexInfiniteRoom* v_xcorr2,
      const double& measurement,
      const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_ycorr1
   * @param v_ycorr2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeYInfiniteRoomYInfiniteRoom* add_y_infinite_room_y_infinite_room_edge(
      g2o::VertexInfiniteRoom* v_ycorr1,
      g2o::VertexInfiniteRoom* v_ycorr2,
      const double& measurement,
      const Eigen::MatrixXd& information);

  g2o::EdgeWall2Planes* add_wall_2planes_edge(g2o::VertexWallXYZ* v_wall,
                                              g2o::VertexPlane* v_plane1,
                                              g2o::VertexPlane* v_plane2,
                                              Eigen::Vector3d wall_point,
                                              const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param edge
   * @param kernel_type
   * @param kernel_size
   */
  void add_robust_kernel(g2o::HyperGraph::Edge* edge,
                         const std::string& kernel_type,
                         double kernel_size);

  /**
   * @brief Perform graph optimization
   *
   * @param num_iterations
   * @return
   */
  int optimize(int num_iterations);

  /**
   * @brief
   *
   * @param spinv
   * @param vert_pairs_vec
   * @return Success or failure
   */
  bool compute_landmark_marginals(g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv,
                                  std::vector<std::pair<int, int>> vert_pairs_vec);

  /**
   * @brief Save the pose graph to a file
   *
   * @param filename:  output filename
   */
  void save(const std::string& filename);

  /**
   * @brief Load the pose graph from file
   *
   * @param filename: output filename
   */
  bool load(const std::string& filename);

 public:
  g2o::RobustKernelFactory* robust_kernel_factory;
  std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph
  int nbr_of_vertices;
  int nbr_of_edges;
  int timing_counter;
  double sum_prev_timings;
  bool save_compute_time;
  std::ofstream time_recorder;
};

}  // namespace s_graphs

#endif  // GRAPH_SLAM_HPP
