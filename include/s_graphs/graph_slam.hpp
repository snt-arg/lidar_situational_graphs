// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>
#include <ros/time.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>

#include <g2o/core/hyper_graph.h>
#include <g2o/vertex_wall.hpp>
#include <g2o/vertex_doorway.hpp>
#include <g2o/edge_wall_two_planes.hpp>
#include <g2o/vertex_doorway.hpp>
#include <g2o/edge_doorway_two_rooms.hpp>

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class VertexInfiniteRoom;
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
class EdgeRoomXPlane;
class EdgeRoom2Planes;
class EdgeRoom4Planes;
class EdgeRoomXPrior;
class EdgeRoomYPlane;
class EdgeRoomYPrior;
class EdgeRoomRoom;
class EdgeRoomXInfiniteRoom;
class EdgeRoomYInfiniteRoom;
class EdgeXInfiniteRoomXInfiniteRoom;
class EdgeYInfiniteRoomYInfiniteRoom;
class EdgePlanePerpendicular;
class Edge2Planes;
class EdgePlanePriorNormal;
class EdgePlanePriorDistance;
class RobustKernelFactory;
class VertexRoomXYLB;
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
  GraphSLAM(const std::string& solver_type = "lm_var");
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
  bool remove_room_node(g2o::VertexRoomXYLB* room_vertex);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return Registered node
   */
  g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz);

  /**
   * @brief Add a infinite_room node to the graph
   *
   * @param infinite_room_pose
   * @return Registered node
   */
  g2o::VertexInfiniteRoom* add_infinite_room_node(const double& infinite_room_pose);

  /**
   * @brief Add a room node to the graph
   *
   * @param room_pose
   * @return Registered node
   */
  g2o::VertexRoomXYLB* add_room_node(const Eigen::Vector2d& room_pose);

  /**
   * @brief Add a floor node to the graph
   *
   * @param floor_pose
   * @return Registered node
   */
  g2o::VertexRoomXYLB* add_floor_node(const Eigen::Vector2d& floor_pose);

  /**
   * @brief Update the floor node estimate in the graph
   *
   * @param floor_node
   * @para floor_pose
   * @return Registered node
   */
  void update_floor_node(g2o::VertexRoomXYLB* floor_node, const Eigen::Vector2d& floor_pose);

  /**
   * @brief Add an edge between SE3 nodes
   *
   * @param v1: node1
   * @param v2: node2
   * @param relative_pose: relative pose between node1 and node2
   * @param information_matrix: information matrix (it must be 6x6)
   * @return registered edge
   */

  g2o::VertexWallXYZ* add_wall_node(const Eigen::Vector3d& wall_center);
  /**
   * @brief add a Wall node to the graph
   * @param wall_center
   * @return registered node
   */

  g2o::VertexDoorWayXYZ* add_doorway_node(const Eigen::Vector3d& door_position);
  /**
   * @brief add a doorway node to the graph
   * @param door_position
   * @return registered node
   */

  g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Add an edge between an SE3 node and a plane node
   *
   * @param v_se3: SE3 node
   * @param v_plane: plane node
   * @param plane_coeffs: plane coefficients w.r.t. v_se3
   * @param information_matrix: information matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Remove a plane edge from the graph
   *
   * @param se3_plane_edge
   * @return Succes or failure
   */
  bool remove_se3_plane_edge(g2o::EdgeSE3Plane* se3_plane_edge);

  /**
   * @brief Add an edge between an SE3 node and to a plane using point to plane distances
   *
   * @param v_se3: SE3 node
   * @param v_plane: plane node
   * @param points_matrix:  plane coefficients w.r.t. v_se3
   * @param information_matrix: information matrix (it must be 1x1)
   * @return registered edge
   */
  g2o::EdgeSE3PointToPlane* add_se3_point_to_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Matrix4d& points_matrix, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Add an edge between an SE3 node and a point_xyz node
   *
   * @param v_se3: SE3 node
   * @param v_xyz: point_xyz node
   * @param xyz: xyz coordinate
   * @param information:  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief Add a prior edge to an SE3 node
   *
   * @param v: Plane
   * @param normal
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v: Plane
   * @param distance
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: Node
   * @param xy
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: Node
   * @param xyz
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3: node
   * @param quat: Quarternion
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_se3
   * @param direction
   * @param measurement
   * @param information_matrix
   * @return registered edge
   */
  g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlane* add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlaneIdentity* add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlaneParallel* add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_plane1
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information);

  g2o::Edge2Planes* add_2planes_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_se3
   * @param v_infinite_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeSE3InfiniteRoom* add_se3_infinite_room_edge(g2o::VertexSE3* v_se3, g2o::VertexInfiniteRoom* v_infinite_room, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_infinite_room
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeInfiniteRoomXPlane* add_infinite_room_xplane_edge(g2o::VertexInfiniteRoom* v_infinite_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_infinite_room
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeInfiniteRoomYPlane* add_infinite_room_yplane_edge(g2o::VertexInfiniteRoom* v_infinite_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param infinite_room_xplane_edge
   * @return Success or failure
   */
  bool remove_infinite_room_xplane_edge(g2o::EdgeInfiniteRoomXPlane* infinite_room_xplane_edge);

  /**
   * @brief
   *
   * @param infinite_room_yplane_edge
   * @return Success or failure
   */
  bool remove_infinite_room_yplane_edge(g2o::EdgeInfiniteRoomYPlane* infinite_room_yplane_edge);

  /**
   * @brief
   *
   * @param v_se3
   * @param v_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeSE3Room* add_se3_room_edge(g2o::VertexSE3* v_se3, g2o::VertexRoomXYLB* v_room, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomXPlane* add_room_xplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

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
  g2o::EdgeRoom2Planes* add_room_2planes_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, g2o::VertexRoomXYLB* v_cluster_center, const Eigen::MatrixXd& information);

  bool remove_room_2planes_edge(g2o::EdgeRoom2Planes* room_plane_edge);

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
  g2o::EdgeRoom4Planes* add_room_4planes_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_xplane1, g2o::VertexPlane* v_xplane2, g2o::VertexPlane* v_yplane1, g2o::VertexPlane* v_yplane2, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomXPrior* add_room_xprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param v_plane2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomYPlane* add_room_yplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomYPrior* add_room_yprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room1
   * @param v_room2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomRoom* add_room_room_edge(g2o::VertexRoomXYLB* v_room1, g2o::VertexRoomXYLB* v_room2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param room_room_edge
   * @return Succes or failure
   */
  bool remove_room_room_edge(g2o::EdgeRoomRoom* room_room_edge);

  /**
   * @brief
   *
   * @param v_room
   * @param v_xinfinite_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomXInfiniteRoom* add_room_x_infinite_room_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexInfiniteRoom* v_xinfinite_room, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_room
   * @param v_yinfinite_room
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeRoomYInfiniteRoom* add_room_y_infinite_room_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexInfiniteRoom* v_yinfinite_room, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_xcorr1
   * @param v_xcorr2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeXInfiniteRoomXInfiniteRoom* add_x_infinite_room_x_infinite_room_edge(g2o::VertexInfiniteRoom* v_xcorr1, g2o::VertexInfiniteRoom* v_xcorr2, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_ycorr1
   * @param v_ycorr2
   * @param measurement
   * @param information
   * @return registered edge
   */
  g2o::EdgeYInfiniteRoomYInfiniteRoom* add_y_infinite_room_y_infinite_room_edge(g2o::VertexInfiniteRoom* v_ycorr1, g2o::VertexInfiniteRoom* v_ycorr2, const double& measurement, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param room_xplane_edge
   * @return Success or failure
   */
  bool remove_room_xplane_edge(g2o::EdgeRoomXPlane* room_xplane_edge);

  /**
   * @brief
   *
   * @param room_yplane_edge
   * @return Success or failure
   */
  bool remove_room_yplane_edge(g2o::EdgeRoomYPlane* room_yplane_edge);

  /**
   * @brief
   *
   * @param v_wall
   * @param v_wall_surface1
   * @param v_wall_surface2
   * @param wall_center
   * @param information
   * @return registered edge
   */
  g2o::EdgeWall2Planes* add_wall_2planes_edge(g2o::VertexWallXYZ* v_wall, g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, Eigen::Vector3d wall_point, const Eigen::MatrixXd& information);

  /**
   * @brief
   *
   * @param v_door_r1
   * @param v_door_r2
   * @param v_room1
   * @param v_room2
   * @param information
   * @return registered edge
   */
  g2o::EdgeDoorWay2Rooms* add_doorway_2rooms_edge(g2o::VertexDoorWayXYZ* v_door_r1, g2o::VertexDoorWayXYZ* v_door_r2, g2o::VertexRoomXYLB* v_room1, g2o::VertexRoomXYLB* v_room2, const Eigen::MatrixXd& information);
  /**
   * @brief
   *
   * @param edge
   * @param kernel_type
   * @param kernel_size
   */
  void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size);

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
  bool compute_landmark_marginals(g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv, std::vector<std::pair<int, int>> vert_pairs_vec);

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
};

}  // namespace s_graphs

#endif  // GRAPH_SLAM_HPP
