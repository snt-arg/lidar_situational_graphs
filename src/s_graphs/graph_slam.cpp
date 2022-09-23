// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/graph_slam.hpp>

#include <boost/format.hpp>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/vertex_room.hpp>
#include <g2o/vertex_corridor.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/robust_kernel_io.hpp>
#include <g2o/edge_corridor_plane.hpp>
#include <g2o/edge_room.hpp>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // be aware of that cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // be aware of that csparse brings LGPL unless it is dynamically linked

namespace g2o {
G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
G2O_REGISTER_TYPE(EDGE_SE3_POINT_TO_PLANE, EdgeSE3PointToPlane)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_NORMAL, EdgePlanePriorNormal)
G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_DISTANCE, EdgePlanePriorDistance)
G2O_REGISTER_TYPE(EDGE_PLANE_IDENTITY, EdgePlaneIdentity)
G2O_REGISTER_TYPE(EDGE_PLANE_PARALLEL, EdgePlaneParallel)
G2O_REGISTER_TYPE(EDGE_PLANE_PAERPENDICULAR, EdgePlanePerpendicular)
G2O_REGISTER_TYPE(EDGE_SE3_CORRIDOR, EdgeSE3Corridor)
G2O_REGISTER_TYPE(EDGE_CORRIDOR_XPLANE, EdgeCorridorXPlane)
G2O_REGISTER_TYPE(EDGE_CORRIDOR_YPLANE, EdgeCorridorYPlane)
G2O_REGISTER_TYPE(EDGE_SE3_ROOM, EdgeSE3Room)
G2O_REGISTER_TYPE(EDGE_ROOM_XPLANE, EdgeRoomXPlane)
G2O_REGISTER_TYPE(EDGE_ROOM_XPRIOR, EdgeRoomXPrior)
G2O_REGISTER_TYPE(EDGE_ROOM_YPLANE, EdgeRoomYPlane)
G2O_REGISTER_TYPE(EDGE_ROOM_YPRIOR, EdgeRoomYPrior)
G2O_REGISTER_TYPE(EDGE_ROOM_ROOM, EdgeRoomRoom)
G2O_REGISTER_TYPE(EDGE_ROOM_XCORRIDOR, EdgeRoomXCorridor)
G2O_REGISTER_TYPE(EDGE_ROOM_YCORRIDOR, EdgeRoomYCorridor)
G2O_REGISTER_TYPE(EDGE_XCORRIDOR_XCORRIDOR, EdgeXCorridorXCorridor)
G2O_REGISTER_TYPE(EDGE_YCORRIDOR_YCORRIDOR, EdgeYCorridorYCorridor)
G2O_REGISTER_TYPE(VERTEX_ROOMXYLB, VertexRoomXYLB)
G2O_REGISTER_TYPE(VERTEX_CORRIDOR, VertexCorridor)
}  // namespace g2o

namespace s_graphs {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM(const std::string& solver_type) {
  graph.reset(new g2o::SparseOptimizer());
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if(!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

  robust_kernel_factory = g2o::RobustKernelFactory::instance();
  vertex_count = edge_count = 0;
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() {
  graph.reset();
}

void GraphSLAM::set_solver(const std::string& solver_type) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if(!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;
}

int GraphSLAM::num_vertices() const {
  return graph->vertices().size();
}
int GraphSLAM::num_edges() const {
  return graph->edges().size();
}

int GraphSLAM::num_vertices_local() const {
  return vertex_count;
}
int GraphSLAM::num_edges_local() const {
  return edge_count;
}

int GraphSLAM::add_vertices() {
  return vertex_count += 1;
}

g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(pose);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

bool GraphSLAM::remove_plane_node(g2o::VertexPlane* plane_vertex) {
  bool ack = graph->removeVertex(plane_vertex);

  return ack;
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

g2o::VertexCorridor* GraphSLAM::add_corridor_node(const double& corridor_pose) {
  g2o::VertexCorridor* vertex(new g2o::VertexCorridor());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(corridor_pose);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

g2o::VertexRoomXYLB* GraphSLAM::add_room_node(const Eigen::Vector2d& room_pose) {
  g2o::VertexRoomXYLB* vertex(new g2o::VertexRoomXYLB());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(room_pose);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

g2o::VertexRoomXYLB* GraphSLAM::add_floor_node(const Eigen::Vector2d& floor_pose) {
  g2o::VertexRoomXYLB* vertex(new g2o::VertexRoomXYLB());
  vertex->setId(static_cast<int>(num_vertices_local()));
  vertex->setEstimate(floor_pose);
  graph->addVertex(vertex);
  this->add_vertices();

  return vertex;
}

g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_se3_plane_edge(g2o::EdgeSE3Plane* se3_plane_edge) {
  bool ack = graph->removeEdge(se3_plane_edge);

  return ack;
}

g2o::EdgeSE3PointToPlane* GraphSLAM::add_se3_point_to_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Matrix4d& points_matrix, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointToPlane* edge(new g2o::EdgeSE3PointToPlane());
  edge->setMeasurement(points_matrix);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePriorNormal* GraphSLAM::add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
  edge->setMeasurement(normal);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePriorDistance* GraphSLAM::add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
  edge->setMeasurement(distance);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
  Eigen::Matrix<double, 6, 1> m;
  m.head<3>() = direction;
  m.tail<3>() = measurement;

  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  edge->setMeasurement(m);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlane* GraphSLAM::add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlane* edge(new g2o::EdgePlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlaneIdentity* GraphSLAM::add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) {
  g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlaneParallel* GraphSLAM::add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgePlanePerpendicular* GraphSLAM::add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Corridor* GraphSLAM::add_se3_corridor_edge(g2o::VertexSE3* v_se3, g2o::VertexCorridor* v_corridor, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeSE3Corridor* edge(new g2o::EdgeSE3Corridor());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_corridor;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeCorridorXPlane* GraphSLAM::add_corridor_xplane_edge(g2o::VertexCorridor* v_corridor, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeCorridorXPlane* edge(new g2o::EdgeCorridorXPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_corridor;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeCorridorYPlane* GraphSLAM::add_corridor_yplane_edge(g2o::VertexCorridor* v_corridor, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeCorridorYPlane* edge(new g2o::EdgeCorridorYPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_corridor;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_corridor_xplane_edge(g2o::EdgeCorridorXPlane* corridor_xplane_edge) {
  bool ack = graph->removeEdge(corridor_xplane_edge);

  return ack;
}

bool GraphSLAM::remove_corridor_yplane_edge(g2o::EdgeCorridorYPlane* corridor_yplane_edge) {
  bool ack = graph->removeEdge(corridor_yplane_edge);

  return ack;
}

g2o::EdgeSE3Room* GraphSLAM::add_se3_room_edge(g2o::VertexSE3* v_se3, g2o::VertexRoomXYLB* v_room, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeSE3Room* edge(new g2o::EdgeSE3Room());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomXPlane* GraphSLAM::add_room_xplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomXPlane* edge(new g2o::EdgeRoomXPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomXPrior* GraphSLAM::add_room_xprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomXPrior* edge(new g2o::EdgeRoomXPrior());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomYPlane* GraphSLAM::add_room_yplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomYPlane* edge(new g2o::EdgeRoomYPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomYPrior* GraphSLAM::add_room_yprior_edge(g2o::VertexRoomXYLB* v_room, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomYPrior* edge(new g2o::EdgeRoomYPrior());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomRoom* GraphSLAM::add_room_room_edge(g2o::VertexRoomXYLB* v_room1, g2o::VertexRoomXYLB* v_room2, const Eigen::Vector2d& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomRoom* edge(new g2o::EdgeRoomRoom());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room1;
  edge->vertices()[1] = v_room2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomXCorridor* GraphSLAM::add_room_x_corridor_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexCorridor* v_xcorridor, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomXCorridor* edge(new g2o::EdgeRoomXCorridor());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_xcorridor;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomYCorridor* GraphSLAM::add_room_y_corridor_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexCorridor* v_ycorridor, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomYCorridor* edge(new g2o::EdgeRoomYCorridor());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_ycorridor;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeXCorridorXCorridor* GraphSLAM::add_x_corridor_x_corridor_edge(g2o::VertexCorridor* v_xcorr1, g2o::VertexCorridor* v_xcorr2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeXCorridorXCorridor* edge(new g2o::EdgeXCorridorXCorridor());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_xcorr1;
  edge->vertices()[1] = v_xcorr2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeYCorridorYCorridor* GraphSLAM::add_y_corridor_y_corridor_edge(g2o::VertexCorridor* v_ycorr1, g2o::VertexCorridor* v_ycorr2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeYCorridorYCorridor* edge(new g2o::EdgeYCorridorYCorridor());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_ycorr1;
  edge->vertices()[1] = v_ycorr2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_room_xplane_edge(g2o::EdgeRoomXPlane* room_xplane_edge) {
  bool ack = graph->removeEdge(room_xplane_edge);

  return ack;
}

bool GraphSLAM::remove_room_yplane_edge(g2o::EdgeRoomYPlane* room_yplane_edge) {
  bool ack = graph->removeEdge(room_yplane_edge);

  return ack;
}

void GraphSLAM::add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) {
  if(kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if(kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  kernel->setDelta(kernel_size);

  g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
  edge_->setRobustKernel(kernel);
}

int GraphSLAM::optimize(int num_iterations) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
  if(graph->edges().size() < 10) {
    return -1;
  }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  graph->initializeOptimization();
  graph->setVerbose(true);

  std::cout << "chi2" << std::endl;
  double chi2 = graph->chi2();

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::WallTime::now();
  int iterations = graph->optimize(num_iterations);
  auto t2 = ros::WallTime::now();
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  return iterations;
}

bool GraphSLAM::compute_landmark_marginals(g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv, std::vector<std::pair<int, int>> vert_pairs_vec) {
  if(graph->computeMarginals(spinv, vert_pairs_vec)) {
    return true;
  } else {
    return false;
  }
}

void GraphSLAM::save(const std::string& filename) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::ofstream ofs(filename);
  graph->save(ofs);

  g2o::save_robust_kernels(filename + ".kernels", graph);
}

bool GraphSLAM::load(const std::string& filename) {
  std::cout << "loading pose graph..." << std::endl;
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::ifstream ifs(filename);
  if(!graph->load(ifs, true)) {
    return false;
  }

  std::cout << "nodes  : " << graph->vertices().size() << std::endl;
  std::cout << "edges  : " << graph->edges().size() << std::endl;

  if(!g2o::load_robust_kernels(filename + ".kernels", graph)) {
    return false;
  }

  return true;
}

}  // namespace s_graphs
