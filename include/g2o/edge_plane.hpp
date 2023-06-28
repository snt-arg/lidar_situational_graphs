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

#ifndef EDGE_PLANE_PARALLEL_HPP
#define EDGE_PLANE_PARALLEL_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>

namespace g2o {

class EdgePlaneParallel
    : public BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlaneParallel() : BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane>() {
    _information.setIdentity();
    _error.setZero();
  }

  void computeError() override;
  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 3; }
};

class EdgePlanePerpendicular
    : public BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlanePerpendicular()
      : BaseBinaryEdge<1, Eigen::Vector3d, VertexPlane, VertexPlane>() {
    _information.setIdentity();
    _error.setZero();
  }

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 3; }
};

class Edge2Planes
    : public BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPlane, g2o::VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Edge2Planes()
      : BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPlane, g2o::VertexPlane>() {
    // _information.setIdentity();
    _error.setZero();
  }

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;
};

}  // namespace g2o

#endif  // EDGE_PLANE_PARALLEL_HPP
