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

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <boost/format.hpp>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/robust_kernel_io.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <g2o/vertex_wall.hpp>
#include <s_graphs/backend/graph_slam.hpp>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // be aware of that cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY(
    csparse)  // be aware of that csparse brings LGPL unless it is dynamically linked

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
G2O_REGISTER_TYPE(EDGE_ROOM_2PLANES, EdgeRoom2Planes)
G2O_REGISTER_TYPE(EDGE_ROOM_4PLANES, EdgeRoom4Planes)
G2O_REGISTER_TYPE(EDGE_FLOOR_ROOM, EdgeFloorRoom)
G2O_REGISTER_TYPE(EDGE_ROOM_ROOM, EdgeRoomRoom)
G2O_REGISTER_TYPE(EDGE_XINFINITE_ROOM_XINFINITE_ROOM, EdgeXInfiniteRoomXInfiniteRoom)
G2O_REGISTER_TYPE(EDGE_YINFINITE_ROOM_YINFINITE_ROOM, EdgeYInfiniteRoomYInfiniteRoom)
G2O_REGISTER_TYPE(VERTEX_ROOM, VertexRoom)
G2O_REGISTER_TYPE(VERTEX_FLOOR, VertexFloor)
G2O_REGISTER_TYPE(VERTEX_INFINITE_ROOM, VertexInfiniteRoom)
}  // namespace g2o

namespace s_graphs {

/**
 * @brief constructor
 */
GraphSLAM::GraphSLAM(const std::string& solver_type, bool save_time) {
  graph.reset(new g2o::SparseOptimizer());
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver =
      solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
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
  timing_counter = 0;
  sum_prev_timings = 0.0;

  save_compute_time = save_time;
  if (save_compute_time) {
    time_recorder.open("/tmp/optimization_computation_time.txt");
    time_recorder << "#time \n";
    time_recorder.close();
  }
}

/**
 * @brief destructor
 */
GraphSLAM::~GraphSLAM() { graph.reset(); }

void GraphSLAM::select_solver_type(const std::string& solver_type) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

  std::cout << "construct solver: " << solver_type << std::endl;
  g2o::OptimizationAlgorithmFactory* solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver =
      solver_factory->construct(solver_type, solver_property);
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
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

int GraphSLAM::retrive_total_nbr_of_edges() const { return graph->edges().size(); }

int GraphSLAM::retrieve_local_nbr_of_vertices() const { return nbr_of_vertices; }
int GraphSLAM::retrieve_local_nbr_of_edges() const { return nbr_of_edges; }

int GraphSLAM::increment_local_nbr_of_vertices() { return nbr_of_vertices += 1; }

int GraphSLAM::increment_local_nbr_of_edges() { return nbr_of_edges += 1; }

g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexSE3* GraphSLAM::copy_se3_node(const g2o::VertexSE3* node) {
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(node->id());
  vertex->setEstimate(node->estimate());
  if (node->fixed()) vertex->setFixed(true);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
  return add_plane_node(plane_coeffs, retrieve_local_nbr_of_vertices());
}

g2o::VertexPlane* GraphSLAM::add_plane_node(const Eigen::Vector4d& plane_coeffs,
                                            const int id) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(id);
  vertex->setEstimate(plane_coeffs);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexPlane* GraphSLAM::copy_plane_node(const g2o::VertexPlane* node) {
  g2o::VertexPlane* vertex(new g2o::VertexPlane());
  vertex->setId(node->id());
  vertex->setEstimate(node->estimate());
  if (node->fixed()) vertex->setFixed(true);
  graph->addVertex(vertex);

  return vertex;
}

bool GraphSLAM::remove_plane_node(g2o::VertexPlane* plane_vertex) {
  return graph->removeVertex(plane_vertex);
}

bool GraphSLAM::remove_room_node(g2o::VertexRoom* room_vertex) {
  return graph->removeVertex(room_vertex);
}

g2o::VertexPointXYZ* GraphSLAM::add_point_xyz_node(const Eigen::Vector3d& xyz) {
  g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(xyz);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexRoom* GraphSLAM::add_room_node(const Eigen::Isometry3d& room_pose) {
  g2o::VertexRoom* vertex(new g2o::VertexRoom());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(room_pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexRoom* GraphSLAM::copy_room_node(const g2o::VertexRoom* node) {
  g2o::VertexRoom* vertex(new g2o::VertexRoom());
  vertex->setId(node->id());
  vertex->setEstimate(node->estimate());
  if (node->fixed()) vertex->setFixed(true);
  graph->addVertex(vertex);

  return vertex;
}

g2o::VertexFloor* GraphSLAM::add_floor_node(const Eigen::Isometry3d& floor_pose) {
  g2o::VertexFloor* vertex(new g2o::VertexFloor());
  vertex->setId(static_cast<int>(retrieve_local_nbr_of_vertices()));
  vertex->setEstimate(floor_pose);
  graph->addVertex(vertex);
  this->increment_local_nbr_of_vertices();

  return vertex;
}

g2o::VertexFloor* GraphSLAM::copy_floor_node(const g2o::VertexFloor* node) {
  g2o::VertexFloor* vertex(new g2o::VertexFloor());
  vertex->setId(node->id());
  vertex->setEstimate(node->estimate());
  if (node->fixed()) vertex->setFixed(true);
  graph->addVertex(vertex);

  return vertex;
}

void GraphSLAM::update_floor_node(g2o::VertexFloor* floor_node,
                                  const Eigen::Isometry3d& floor_pose) {
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

g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1,
                                      g2o::VertexSE3* v2,
                                      const Eigen::Isometry3d& relative_pose,
                                      const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3* GraphSLAM::copy_se3_edge(g2o::EdgeSE3* e,
                                       g2o::VertexSE3* v1,
                                       g2o::VertexSE3* v2) {
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setId(e->id());
  edge->setMeasurement(e->measurement());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Plane* GraphSLAM::add_se3_plane_edge(
    g2o::VertexSE3* v_se3,
    g2o::VertexPlane* v_plane,
    const Eigen::Vector4d& plane_coeffs,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(plane_coeffs);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3Plane* GraphSLAM::copy_se3_plane_edge(g2o::EdgeSE3Plane* e,
                                                  g2o::VertexSE3* v1,
                                                  g2o::VertexPlane* v2) {
  g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
  edge->setId(e->id());
  edge->setMeasurement(e->measurement());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_se3_plane_edge(g2o::EdgeSE3Plane* se3_plane_edge) {
  bool ack = graph->removeEdge(se3_plane_edge);

  return ack;
}

void GraphSLAM::update_se3edge_information(g2o::EdgeSE3* edge_se3,
                                           Eigen::MatrixXd information_matrix) {
  edge_se3->setInformation(information_matrix);
}

g2o::EdgeSE3PointToPlane* GraphSLAM::add_se3_point_to_plane_edge(
    g2o::VertexSE3* v_se3,
    g2o::VertexPlane* v_plane,
    const Eigen::Matrix4d& points_matrix,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointToPlane* edge(new g2o::EdgeSE3PointToPlane());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(points_matrix);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_plane;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3PointXYZ* GraphSLAM::add_se3_point_xyz_edge(
    g2o::VertexSE3* v_se3,
    g2o::VertexPointXYZ* v_xyz,
    const Eigen::Vector3d& xyz,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_xyz;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlanePriorNormal* GraphSLAM::add_plane_normal_prior_edge(
    g2o::VertexPlane* v,
    const Eigen::Vector3d& normal,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(normal);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlanePriorDistance* GraphSLAM::add_plane_distance_prior_edge(
    g2o::VertexPlane* v,
    double distance,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(distance);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(
    g2o::VertexSE3* v_se3,
    const Eigen::Vector2d& xy,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(xy);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(
    g2o::VertexSE3* v_se3,
    const Eigen::Vector3d& xyz,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(
    g2o::VertexSE3* v_se3,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& measurement,
    const Eigen::MatrixXd& information_matrix) {
  Eigen::Matrix<double, 6, 1> m;
  m.head<3>() = direction;
  m.tail<3>() = measurement;

  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(m);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(
    g2o::VertexSE3* v_se3,
    const Eigen::Quaterniond& quat,
    const Eigen::MatrixXd& information_matrix) {
  g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlane* GraphSLAM::add_plane_edge(g2o::VertexPlane* v_plane1,
                                          g2o::VertexPlane* v_plane2,
                                          const Eigen::Vector4d& measurement,
                                          const Eigen::Matrix4d& information) {
  g2o::EdgePlane* edge(new g2o::EdgePlane());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlaneIdentity* GraphSLAM::add_plane_identity_edge(
    g2o::VertexPlane* v_plane1,
    g2o::VertexPlane* v_plane2,
    const Eigen::Vector4d& measurement,
    const Eigen::Matrix4d& information) {
  g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlaneParallel* GraphSLAM::add_plane_parallel_edge(
    g2o::VertexPlane* v_plane1,
    g2o::VertexPlane* v_plane2,
    const Eigen::Vector3d& measurement,
    const Eigen::MatrixXd& information) {
  g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgePlanePerpendicular* GraphSLAM::add_plane_perpendicular_edge(
    g2o::VertexPlane* v_plane1,
    g2o::VertexPlane* v_plane2,
    const Eigen::Vector3d& measurement,
    const Eigen::MatrixXd& information) {
  g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::Edge2Planes* GraphSLAM::add_2planes_edge(g2o::VertexPlane* v_plane1,
                                              g2o::VertexPlane* v_plane2,
                                              const Eigen::MatrixXd& information) {
  g2o::Edge2Planes* edge(new g2o::Edge2Planes());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setInformation(information);
  edge->vertices()[0] = v_plane1;
  edge->vertices()[1] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::Edge2Planes* GraphSLAM::copy_2planes_edge(g2o::Edge2Planes* e,
                                               g2o::VertexPlane* v1,
                                               g2o::VertexPlane* v2) {
  g2o::Edge2Planes* edge(new g2o::Edge2Planes());
  edge->setId(e->id());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeSE3Room* GraphSLAM::add_se3_room_edge(g2o::VertexSE3* v_se3,
                                               g2o::VertexRoom* v_room,
                                               const Eigen::Isometry3d& measurement,
                                               const Eigen::MatrixXd& information) {
  g2o::EdgeSE3Room* edge(new g2o::EdgeSE3Room());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_se3;
  edge->vertices()[1] = v_room;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeRoom2Planes* GraphSLAM::add_room_2planes_edge(
    g2o::VertexRoom* v_room,
    g2o::VertexPlane* v_plane1,
    g2o::VertexPlane* v_plane2,
    g2o::VertexRoom* v_cluster_center,
    const Eigen::MatrixXd& information) {
  g2o::EdgeRoom2Planes* edge(new g2o::EdgeRoom2Planes());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_plane1;
  edge->vertices()[2] = v_plane2;
  edge->vertices()[3] = v_cluster_center;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeWall2Planes* GraphSLAM::add_wall_2planes_edge(
    g2o::VertexWallXYZ* v_wall,
    g2o::VertexPlane* v_plane1,
    g2o::VertexPlane* v_plane2,
    Eigen::Vector3d wall_point,
    const Eigen::MatrixXd& information) {
  g2o::EdgeWall2Planes* edge(new g2o::EdgeWall2Planes(wall_point));
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setInformation(information);
  edge->vertices()[0] = v_wall;
  edge->vertices()[1] = v_plane1;
  edge->vertices()[2] = v_plane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

bool GraphSLAM::remove_room_2planes_edge(g2o::EdgeRoom2Planes* room_plane_edge) {
  bool ack = graph->removeEdge(room_plane_edge);

  return ack;
}

g2o::EdgeRoom2Planes* GraphSLAM::copy_room_2planes_edge(g2o::EdgeRoom2Planes* e,
                                                        g2o::VertexRoom* v1,
                                                        g2o::VertexPlane* v2,
                                                        g2o::VertexPlane* v3,
                                                        g2o::VertexRoom* v4) {
  g2o::EdgeRoom2Planes* edge(new g2o::EdgeRoom2Planes());
  edge->setId(e->id());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  edge->vertices()[2] = v3;
  edge->vertices()[3] = v4;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeRoom4Planes* GraphSLAM::add_room_4planes_edge(
    g2o::VertexRoom* v_room,
    g2o::VertexPlane* v_xplane1,
    g2o::VertexPlane* v_xplane2,
    g2o::VertexPlane* v_yplane1,
    g2o::VertexPlane* v_yplane2,
    const Eigen::MatrixXd& information) {
  g2o::EdgeRoom4Planes* edge(new g2o::EdgeRoom4Planes());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setInformation(information);
  edge->vertices()[0] = v_room;
  edge->vertices()[1] = v_xplane1;
  edge->vertices()[2] = v_xplane2;
  edge->vertices()[3] = v_yplane1;
  edge->vertices()[4] = v_yplane2;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeRoom4Planes* GraphSLAM::copy_room_4planes_edge(g2o::EdgeRoom4Planes* e,
                                                        g2o::VertexRoom* v1,
                                                        g2o::VertexPlane* v2,
                                                        g2o::VertexPlane* v3,
                                                        g2o::VertexPlane* v4,
                                                        g2o::VertexPlane* v5) {
  g2o::EdgeRoom4Planes* edge(new g2o::EdgeRoom4Planes());
  edge->setId(e->id());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  edge->vertices()[2] = v3;
  edge->vertices()[3] = v4;
  edge->vertices()[4] = v5;
  graph->addEdge(edge);

  return edge;
}

g2o::EdgeFloorRoom* GraphSLAM::add_floor_room_edge(g2o::VertexFloor* v_floor,
                                                   g2o::VertexRoom* v_room,
                                                   const Eigen::Vector2d& measurement,
                                                   const Eigen::MatrixXd& information) {
  g2o::EdgeFloorRoom* edge(new g2o::EdgeFloorRoom());
  edge->setId(static_cast<int>(retrieve_local_nbr_of_edges()));
  edge->setMeasurement(measurement);
  edge->setInformation(information);
  edge->vertices()[0] = v_floor;
  edge->vertices()[1] = v_room;
  graph->addEdge(edge);
  this->increment_local_nbr_of_edges();

  return edge;
}

g2o::EdgeFloorRoom* GraphSLAM::copy_floor_room_edge(g2o::EdgeFloorRoom* e,
                                                    g2o::VertexFloor* v1,
                                                    g2o::VertexRoom* v2) {
  g2o::EdgeFloorRoom* edge(new g2o::EdgeFloorRoom());
  edge->setId(e->id());
  edge->setMeasurement(e->measurement());
  edge->setInformation(e->information());
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph->addEdge(edge);

  return edge;
}

bool GraphSLAM::remove_floor_room_edge(g2o::EdgeFloorRoom* floor_room_edge) {
  bool ack = graph->removeEdge(floor_room_edge);

  return ack;
}

void GraphSLAM::add_robust_kernel(g2o::HyperGraph::Edge* edge,
                                  const std::string& kernel_type,
                                  double kernel_size) {
  if (kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if (kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  kernel->setDelta(kernel_size);

  g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
  edge_->setRobustKernel(kernel);
}

int GraphSLAM::optimize(int num_iterations) {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
  if (graph->edges().size() < 1) {
    return -1;
  }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size()
            << "   edges: " << graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  std::cout << "init" << std::endl;
  graph->initializeOptimization();
  graph->setVerbose(false);

  double chi2 = graph->chi2();
  if (std::isnan(graph->chi2())) {
    std::cout << "GRAPH RETURNED A NAN WAITING AFTER OPTIMIZATION" << std::endl;
  }

  std::cout << "optimize!!" << std::endl;
  auto t1 = rclcpp::Clock{}.now();
  int iterations = graph->optimize(num_iterations);
  auto t2 = rclcpp::Clock{}.now();
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).seconds() << "[sec]"
            << std::endl;
  timing_counter++;
  sum_prev_timings += (t2 - t1).seconds();
  std::cout << "avg Computation Time: "
            << boost::format("%.3f") % (sum_prev_timings / timing_counter) << "[sec]"
            << std::endl;

  if (save_compute_time) {
    time_recorder.open("/tmp/optimization_computation_time.txt",
                       std::ofstream::out | std::ofstream::app);
    time_recorder << std::to_string((t2 - t1).seconds()) + " \n";
    time_recorder.close();
  }

  if (std::isnan(graph->chi2())) {
    throw std::invalid_argument("GRAPH RETURNED A NAN...STOPPING THE EXPERIMENT");
  }

  return iterations;
}

bool GraphSLAM::compute_landmark_marginals(
    g2o::SparseBlockMatrix<Eigen::MatrixXd>& spinv,
    std::vector<std::pair<int, int>> vert_pairs_vec) {
  if (graph->computeMarginals(spinv, vert_pairs_vec)) {
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
  if (!graph->load(ifs)) {
    return false;
  }

  std::cout << "nodes  : " << graph->vertices().size() << std::endl;
  std::cout << "edges  : " << graph->edges().size() << std::endl;

  if (!g2o::load_robust_kernels(filename + ".kernels", graph)) {
    return false;
  }

  return true;
}

}  // namespace s_graphs
