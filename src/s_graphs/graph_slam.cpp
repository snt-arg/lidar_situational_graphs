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
#include <g2o/vertex_wall.hpp>
#include <g2o/vertex_doorway.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/robust_kernel_io.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_doorway_two_rooms.hpp>

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
G2O_REGISTER_TYPE(EDGE_2PLANES, Edge2Planes)
G2O_REGISTER_TYPE(EDGE_SE3_INFINITE_ROOM, EdgeSE3InfiniteRoom)
G2O_REGISTER_TYPE(EDGE_INFINITE_ROOM_XPLANE, EdgeInfiniteRoomXPlane)
G2O_REGISTER_TYPE(EDGE_INFINITE_ROOM_YPLANE, EdgeInfiniteRoomYPlane)
G2O_REGISTER_TYPE(EDGE_SE3_ROOM, EdgeSE3Room)
G2O_REGISTER_TYPE(EDGE_ROOM_XPLANE, EdgeRoomXPlane)
G2O_REGISTER_TYPE(EDGE_ROOM_2PLANES, EdgeRoom2Planes)
G2O_REGISTER_TYPE(EDGE_ROOM_4PLANES, EdgeRoom4Planes)
G2O_REGISTER_TYPE(EDGE_ROOM_XPRIOR, EdgeRoomXPrior)
G2O_REGISTER_TYPE(EDGE_ROOM_YPLANE, EdgeRoomYPlane)
G2O_REGISTER_TYPE(EDGE_ROOM_YPRIOR, EdgeRoomYPrior)
G2O_REGISTER_TYPE(EDGE_ROOM_ROOM, EdgeRoomRoom)
G2O_REGISTER_TYPE(EDGE_ROOM_XINFINITE_ROOM, EdgeRoomXInfiniteRoom)
G2O_REGISTER_TYPE(EDGE_ROOM_YINFINITE_ROOM, EdgeRoomYInfiniteRoom)
G2O_REGISTER_TYPE(EDGE_XINFINITE_ROOM_XINFINITE_ROOM, EdgeXInfiniteRoomXInfiniteRoom)
G2O_REGISTER_TYPE(EDGE_YINFINITE_ROOM_YINFINITE_ROOM, EdgeYInfiniteRoomYInfiniteRoom)
G2O_REGISTER_TYPE(VERTEX_ROOMXYLB, VertexRoomXYLB)
G2O_REGISTER_TYPE(VERTEX_INFINITE_ROOM, VertexInfiniteRoom)
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
  nbr_of_vertices = nbr_of_edges = 0;
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() {
  graph.reset();
}

void GraphSLAM::select_solver_type(const std::string& solver_type) {
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

int GraphSLAM::retrieve_total_nbr_of_vertices() const {
  return graph->vertices().size();
}
int GraphSLAM::retrive_total_nbr_of_edges() const {
  return graph->edges().size();
}

int GraphSLAM::retrieve_local_nbr_of_vertices() const {
  return nbr_of_vertices;
}
int GraphSLAM::retrieve_local_nbr_of_edges() const {
  return nbr_of_edges;
}

int GraphSLAM::increment_local_nbr_of_vertices() {
  return nbr_of_vertices += 1;
}

g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs, const int id) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(id);
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

bool GraphSLAM::remove_plane_node(g2o::VertexPlane* plane_vertex) {
  bool ack = graph->removeVertex(plane_vertex);

  return ack;
}

bool GraphSLAM::remove_room_node(g2o::VertexRoomXYLB* room_vertex) {
  bool ack = graph->removeVertex(room_vertex);

  return ack;
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexInfiniteRoom* GraphSLAM::add_infinite_room_node(const double& infinite_room_pose) {
  g2o::VertexInfiniteRoom* vertex(new g2o::VertexInfiniteRoom());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(infinite_room_pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexRoomXYLB* GraphSLAM::add_room_node(const Eigen::Vector2d& room_pose) {
  g2o::VertexRoomXYLB* vertex(new g2o::VertexRoomXYLB());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(room_pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexRoomXYLB* GraphSLAM::add_floor_node(const Eigen::Vector2d& floor_pose) {
  g2o::VertexRoomXYLB* vertex(new g2o::VertexRoomXYLB());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(floor_pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

void GraphSLAM::update_floor_node(g2o::VertexRoomXYLB* floor_node, const Eigen::Vector2d& floor_pose) {
  floor_node->setEstimate(floor_pose);

  return;
}

g2o::VertexWallXYZ* GraphSLAM::add_wall_node(const Eigen::Vector3d& wall_center) {
  g2o::VertexWallXYZ* vertex(new g2o::VertexWallXYZ());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(wall_center);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexDoorWayXYZ* GraphSLAM::add_doorway_node(const Eigen::Vector3d& door_position) {
  g2o::VertexDoorWayXYZ* vertex(new g2o::VertexDoorWayXYZ());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(door_position);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

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

g2o::Edge2Planes* GraphSLAM::add_2planes_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::MatrixXd& information) {
  g2o::Edge2Planes* edge(new g2o::Edge2Planes());
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3InfiniteRoom* GraphSLAM::add_se3_infinite_room_edge(g2o::VertexSE3* v_se3, g2o::VertexInfiniteRoom* v_infinite_room, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeSE3InfiniteRoom* edge(new g2o::EdgeSE3InfiniteRoom());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_infinite_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeInfiniteRoomXPlane* GraphSLAM::add_infinite_room_xplane_edge(g2o::VertexInfiniteRoom* v_infinite_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeInfiniteRoomXPlane* edge(new g2o::EdgeInfiniteRoomXPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_infinite_room;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeInfiniteRoomYPlane* GraphSLAM::add_infinite_room_yplane_edge(g2o::VertexInfiniteRoom* v_infinite_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeInfiniteRoomYPlane* edge(new g2o::EdgeInfiniteRoomYPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_infinite_room;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_infinite_room_xplane_edge(g2o::EdgeInfiniteRoomXPlane* infinite_room_xplane_edge) {
  bool ack = graph->removeEdge(infinite_room_xplane_edge);

  return ack;
}

bool GraphSLAM::remove_infinite_room_yplane_edge(g2o::EdgeInfiniteRoomYPlane* infinite_room_yplane_edge) {
  bool ack = graph->removeEdge(infinite_room_yplane_edge);

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

g2o::EdgeRoomXPlane* GraphSLAM::add_room_xplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomXPlane* edge(new g2o::EdgeRoomXPlane());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoom2Planes* GraphSLAM::add_room_2planes_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, g2o::VertexRoomXYLB* v_cluster_center, const Eigen::MatrixXd& information) {
  g2o::EdgeRoom2Planes* edge(new g2o::EdgeRoom2Planes());
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_plane1;
  edge->vertices()[2] = v_plane2;
  edge->vertices()[3] = v_cluster_center;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeWall2Planes* GraphSLAM::add_wall_2planes_edge(g2o::VertexWallXYZ* v_wall, g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, Eigen::Vector3d wall_point, const Eigen::MatrixXd& information) {
  g2o::EdgeWall2Planes* edge(new g2o::EdgeWall2Planes(wall_point));
  edge->setInformation(information);
  edge->vertices()[0] = v_wall;
  edge->vertices()[1] = v_plane1;
  edge->vertices()[2] = v_plane2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeDoorWay2Rooms* GraphSLAM::add_doorway_2rooms_edge(g2o::VertexDoorWayXYZ* v_door_r1, g2o::VertexDoorWayXYZ* v_door_r2, g2o::VertexRoomXYLB* v_room1, g2o::VertexRoomXYLB* v_room2, const Eigen::MatrixXd& information) {
  g2o::EdgeDoorWay2Rooms* edge(new g2o::EdgeDoorWay2Rooms());
  edge->setInformation(information);
  edge->vertices()[0] = v_door_r1;
  edge->vertices()[1] = v_door_r2;
  edge->vertices()[2] = v_room1;
  edge->vertices()[3] = v_room2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_room_2planes_edge(g2o::EdgeRoom2Planes* room_plane_edge) {
  bool ack = graph->removeEdge(room_plane_edge);

  return ack;
}

g2o::EdgeRoom4Planes* GraphSLAM::add_room_4planes_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_xplane1, g2o::VertexPlane* v_xplane2, g2o::VertexPlane* v_yplane1, g2o::VertexPlane* v_yplane2, const Eigen::MatrixXd& information) {
  g2o::EdgeRoom4Planes* edge(new g2o::EdgeRoom4Planes());
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_xplane1;
  edge->vertices()[2] = v_xplane2;
  edge->vertices()[3] = v_yplane1;
  edge->vertices()[4] = v_yplane2;
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

g2o::EdgeRoomYPlane* GraphSLAM::add_room_yplane_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexPlane* v_plane2, const double& measurement, const Eigen::MatrixXd& information) {
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

bool GraphSLAM::remove_room_room_edge(g2o::EdgeRoomRoom* room_room_edge) {
  bool ack = graph->removeEdge(room_room_edge);

  return ack;
}

g2o::EdgeRoomXInfiniteRoom* GraphSLAM::add_room_x_infinite_room_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexInfiniteRoom* v_xinfinite_room, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomXInfiniteRoom* edge(new g2o::EdgeRoomXInfiniteRoom());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_xinfinite_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoomYInfiniteRoom* GraphSLAM::add_room_y_infinite_room_edge(g2o::VertexRoomXYLB* v_room, g2o::VertexInfiniteRoom* v_yinfinite_room, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeRoomYInfiniteRoom* edge(new g2o::EdgeRoomYInfiniteRoom());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_yinfinite_room;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeXInfiniteRoomXInfiniteRoom* GraphSLAM::add_x_infinite_room_x_infinite_room_edge(g2o::VertexInfiniteRoom* v_xcorr1, g2o::VertexInfiniteRoom* v_xcorr2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeXInfiniteRoomXInfiniteRoom* edge(new g2o::EdgeXInfiniteRoomXInfiniteRoom());
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_xcorr1;
  edge->vertices()[1] = v_xcorr2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeYInfiniteRoomYInfiniteRoom* GraphSLAM::add_y_infinite_room_y_infinite_room_edge(g2o::VertexInfiniteRoom* v_ycorr1, g2o::VertexInfiniteRoom* v_ycorr2, const double& measurement, const Eigen::MatrixXd& information) {
  g2o::EdgeYInfiniteRoomYInfiniteRoom* edge(new g2o::EdgeYInfiniteRoomYInfiniteRoom());
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

  double chi2 = graph->chi2();
  if(std::isnan(graph->chi2())) {
    std::cout << "GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION" << std::endl;
  }

  std::cout << "optimize!!" << std::endl;
  auto t1 = ros::Time::now();
  int iterations = graph->optimize(num_iterations);
  auto t2 = ros::Time::now();
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

  if(std::isnan(graph->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }

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
