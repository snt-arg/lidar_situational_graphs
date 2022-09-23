// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>
#include <ros/time.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>

#include <g2o/core/hyper_graph.h>

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class VertexCorridor;
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
class EdgeSE3Corridor;
class EdgeCorridorXPlane;
class EdgeCorridorYPlane;
class EdgeSE3Room;
class EdgeRoomXPlane;
class EdgeRoomXPrior;
class EdgeRoomYPlane;
class EdgeRoomYPrior;
class EdgeRoomRoom;
class EdgeRoomXCorridor;
class EdgeRoomYCorridor;
class EdgeXCorridorXCorridor;
class EdgeYCorridorYCorridor;
class EdgePlanePerpendicular;
class EdgePlanePriorNormal;
class EdgePlanePriorDistance;
class RobustKernelFactory;
class VertexRoomXYLB;
}  // namespace g2o

namespace s_graphs {

class GraphSLAM {
public:
  GraphSLAM(const std::string& solver_type = "lm_var");
  virtual ~GraphSLAM();

  int num_vertices() const;
  int num_edges() const;
  int num_vertices_local() const;
  int num_edges_local() const;
  int add_vertices();

  void set_solver(const std::string& solver_type);

  /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
  g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

  /**
   * @brief add a plane node to the graph
   * @param plane_coeffs
   * @return registered node
   */
  g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

  /**
   * @brief remove a plane node from the graph
   * @param plane id
   * @return success or failure
   */
  bool remove_plane_node(g2o::VertexPlane* plane_vertex);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return registered node
   */
  g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz);

  /**
   * @brief add a corridor node to the graph
   * @param corridor
   * @return registered node
   */
  g2o::VertexCorridor* add_corridor_node(const double& corridor_pose);

  /**
   * @brief add a room node to the graph
   * @param room
   * @return registered node
   */
  g2o::VertexRoomXYLB* add_room_node(const Eigen::Vector2d& room_pose);

  /**
   * @brief add a floor node to the graph
   * @param floor
   * @return registered node
   */
  g2o::VertexRoomXYLB* add_floor_node(const Eigen::Vector2d& floor_pose);

  /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
  g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a plane node
   * @param v_se3    SE3 node
   * @param v_plane  plane node
   * @param plane_coeffs  plane coefficients w.r.t. v_se3
   * @param information_matrix  information matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

  bool remove_se3_plane_edge(g2o::EdgeSE3Plane* se3_plane_edge);

  /**
   * @brief add an edge between an SE3 node and to a plane using point to plane distances
   * @param v_se3    SE3 node
   * @param v_plane  plane node
   * @param plane_coeffs  plane coefficients w.r.t. v_se3
   * @param information_matrix  information matrix (it must be 1x1)
   * @return registered edge
   */
  g2o::EdgeSE3PointToPlane* add_se3_point_to_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Matrix4d& points_matrix, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a point_xyz node
   * @param v_se3        SE3 node
   * @param v_xyz        point_xyz node
   * @param xyz          xyz coordinate
   * @param information  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add a prior edge to an SE3 node
   * @param v_se3
   * @param xy
   * @param information_matrix
   * @return
   */
  g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix);

  g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix);

  g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix);

  g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

  g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix);

  g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix);

  g2o::EdgePlane* add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information);

  g2o::EdgePlaneIdentity* add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information);

  g2o::EdgePlaneParallel* add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeSE3Corridor* add_se3_corridor_edge(g2o::VertexSE3* v_se3, g2o::VertexCorridor* v_corridor, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeCorridorXPlane* add_corridor_xplane_edge(g2o::VertexCorridor* v_corridor, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeCorridorYPlane* add_corridor_yplane_edge(g2o::VertexCorridor* v_corridor, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information);

  bool remove_corridor_xplane_edge(g2o::EdgeCorridorXPlane* corridor_xplane_edge);

  bool remove_corridor_yplane_edge(g2o::EdgeCorridorYPlane* corridor_yplane_edge);

  g2o::EdgeSE3Room* add_se3_room_edge(g2o::VertexSE3* v_se3, g2o::VertexRoomXYLB* v_room, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomXPlane* add_room_xplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomXPrior* add_room_xprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomYPlane* add_room_yplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomYPrior* add_room_yprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomRoom* add_room_room_edge(g2o::VertexRoomXYLB* v_room1, g2o::VertexRoomXYLB* v_room2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomXCorridor* add_room_x_corridor_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexCorridor* v_xcorridor, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeRoomYCorridor* add_room_y_corridor_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexCorridor* v_ycorridor, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeXCorridorXCorridor* add_x_corridor_x_corridor_edge(g2o::VertexCorridor* v_xcorr1, g2o::VertexCorridor* v_xcorr2, const double& measurement, const Eigen::MatrixXd& information);

  g2o::EdgeYCorridorYCorridor* add_y_corridor_y_corridor_edge(g2o::VertexCorridor* v_ycorr1, g2o::VertexCorridor* v_ycorr2, const double& measurement, const Eigen::MatrixXd& information);

  bool remove_room_xplane_edge(g2o::EdgeRoomXPlane* room_xplane_edge);

  bool remove_room_yplane_edge(g2o::EdgeRoomYPlane* room_yplane_edge);

  void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size);

  /**
   * @brief perform graph optimization
   */
  int optimize(int num_iterations);

  bool compute_landmark_marginals(g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv, std::vector<std::pair<int, int>> vert_pairs_vec);

  /**
   * @brief save the pose graph to a file
   * @param filename  output filename
   */
  void save(const std::string& filename);

  /**
   * @brief load the pose graph from file
   * @param filename  output filename
   */
  bool load(const std::string& filename);

public:
  g2o::RobustKernelFactory* robust_kernel_factory;
  std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph
  int vertex_count;
  int edge_count;
};

}  // namespace s_graphs

#endif  // GRAPH_SLAM_HPP
