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

#ifndef EDGE_INFINITE_ROOM_PLANE_HPP
#define EDGE_INFINITE_ROOM_PLANE_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>

#include "g2o/vertex_infinite_room.hpp"

namespace g2o {

class EdgeSE3InfiniteRoom
    : public BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexInfiniteRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3InfiniteRoom()
      : BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexInfiniteRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const double& m) override { _measurement = m; }
};

class EdgeInfiniteRoomXPlane
    : public BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInfiniteRoomXPlane()
      : BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexPlane>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const double& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 1; }
};

class EdgeInfiniteRoomYPlane
    : public BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInfiniteRoomYPlane()
      : BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexPlane>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const double& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 1; }
};

class EdgeXInfiniteRoomXInfiniteRoom : public BaseBinaryEdge<1,
                                                             double,
                                                             g2o::VertexInfiniteRoom,
                                                             g2o::VertexInfiniteRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeXInfiniteRoomXInfiniteRoom()
      : BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexInfiniteRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const double& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 1; }
};

class EdgeYInfiniteRoomYInfiniteRoom : public BaseBinaryEdge<1,
                                                             double,
                                                             g2o::VertexInfiniteRoom,
                                                             g2o::VertexInfiniteRoom> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeYInfiniteRoomYInfiniteRoom()
      : BaseBinaryEdge<1, double, g2o::VertexInfiniteRoom, g2o::VertexInfiniteRoom>() {}

  void computeError() override;

  virtual bool read(std::istream& is) override;

  virtual bool write(std::ostream& os) const override;

  virtual void setMeasurement(const double& m) override { _measurement = m; }

  virtual int measurementDimension() const override { return 1; }
};

}  // namespace g2o
#endif  // EDGE_INFINITE_ROOM_PLANE_HPP
