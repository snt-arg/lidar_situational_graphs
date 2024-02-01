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

// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef EDGE_ROOM_HPP
#define EDGE_ROOM_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>

#include "g2o/vertex_floor.hpp"
#include "g2o/vertex_infinite_room.hpp"
#include "g2o/vertex_room.hpp"

#include "s_graphs/backend/factor_nn.hpp"

namespace g2o {

// ADAPTED FROM G2O SE3 edge
class EdgeRoomRoom
    : public BaseBinaryEdge<6, Eigen::Isometry3d, g2o::VertexRoom, g2o::VertexRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomRoom();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  void computeError();

  virtual void setMeasurement(const Isometry3& m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  virtual bool setMeasurementData(const number_t* d) {
    Eigen::Map<const Vector7> v(d);
    setMeasurement(internal::fromVectorQT(v));
    return true;
  }

  virtual bool getMeasurementData(number_t* d) const {
    Eigen::Map<Vector7> v(d);
    v = internal::toVectorQT(_measurement);
    return true;
  }

  void linearizeOplus();

  virtual int measurementDimension() const { return 7; }

  virtual bool setMeasurementFromState();

  virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
                                           OptimizableGraph::Vertex* /*to*/) {
    return 1.;
  }

  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* to);

 protected:
  Isometry3 _inverseMeasurement;
};

class EdgeSE3Room
    : public BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE3, g2o::VertexRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Room()
      : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE3, g2o::VertexRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const Eigen::Vector2d& m) override { _measurement = m; }
};

class EdgeRoom2Planes : public BaseMultiEdge<2, Eigen::Vector2d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoom2Planes() : BaseMultiEdge<2, Eigen::Vector2d>() { resize(4); }

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

 private:
  void correct_plane_direction(Eigen::Vector4d& plane);
};

class EdgeRoom4Planes : public BaseMultiEdge<2, Eigen::Vector2d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoom4Planes(bool use_factor_nn_arg = true) : BaseMultiEdge<2, Eigen::Vector2d>() { resize(5); _use_factor_nn = use_factor_nn_arg;}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  s_graphs::FactorNN factor_nn;

 private:
  void correct_plane_direction(Eigen::Vector4d& plane);
  Eigen::Vector2d compute_factor_legacy(Eigen::Vector4d x_plane1, Eigen::Vector4d x_plane2, Eigen::Vector4d y_plane1, Eigen::Vector4d y_plane2);
  Eigen::Vector2d compute_factor_nn(Eigen::Vector4d x_plane1, Eigen::Vector4d x_plane2, Eigen::Vector4d y_plane1, Eigen::Vector4d y_plane2);
  bool _use_factor_nn;
};

class EdgeFloorRoom
    : public BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexFloor, g2o::VertexRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeFloorRoom()
      : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexFloor, g2o::VertexRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const Eigen::Vector2d& m) override { _measurement = m; }
};

class Edge2Rooms
    : public BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexRoom, g2o::VertexRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Edge2Rooms()
      : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexRoom, g2o::VertexRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;
};

}  // namespace g2o
#endif  // EDGE_ROOM_HPP
