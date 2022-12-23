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

#ifndef G2O_DOOR_WAY_XYZ_H
#define G2O_DOOR_WAY_XYZ_H

//#include "g2o/config.h"
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {

class G2O_TYPES_SLAM3D_API VertexDoorWayXYZ : public BaseVertex<3, Vector3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexDoorWayXYZ() {}

  virtual void setToOriginImpl() {
    _estimate.fill(0.);
  }

  virtual bool setEstimateDataImpl(const number_t* est) {
    Eigen::Map<const Vector3> _est(est);
    _estimate = _est;

    return true;
  }

  virtual bool getEstimateData(number_t* est) const {
    Eigen::Map<Vector3> _est(est);
    _est = _estimate;
    return true;
  }

  virtual int estimateDimension() const {
    return 3;
  }

  virtual bool setMinimalEstimateDataImpl(const number_t* est) {
    _estimate = Eigen::Map<const Vector3>(est);
    return true;
  }

  virtual bool getMinimalEstimateData(number_t* est) const {
    Eigen::Map<Vector3> v(est);
    v = _estimate;
    return true;
  }

  virtual int minimalEstimateDimension() const {
    return 3;
  }

  virtual void oplusImpl(const number_t* update) {
    for(int i = 0; i < 3; i++) _estimate[i] += update[i];
  }

  virtual bool read(std::istream& is) {
    return internal::readVector(is, _estimate);
  }
  virtual bool write(std::ostream& os) const {
    return internal::writeVector(os, estimate());
  }
};

}  // namespace g2o

#endif