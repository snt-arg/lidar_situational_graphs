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

#ifndef G2O_VERTEX_ROOM
#define G2O_VERTEX_ROOM

#include <g2o/types/slam3d/g2o_types_slam3d_api.h>

#include <Eigen/Core>

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

class G2O_TYPES_SLAM3D_API VertexRoom : public BaseVertex<6, Isometry3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexRoom() {
    _numOplusCalls = 0;
    setToOriginImpl();
    updateCache();
  }
  static const int orthogonalizeAfter =
      1000;  //< orthogonalize the rotation matrix after N updates

  virtual void setToOriginImpl() { _estimate = Isometry3::Identity(); }

  virtual bool setEstimateDataImpl(const number_t* est) {
    Eigen::Map<const Vector7> v(est);
    _estimate = internal::fromVectorQT(v);
    return true;
  }

  virtual bool getEstimateData(number_t* est) const {
    Eigen::Map<Vector7> v(est);
    v = internal::toVectorQT(_estimate);
    return true;
  }

  virtual int estimateDimension() const { return 7; }

  virtual bool setMinimalEstimateDataImpl(const number_t* est) {
    Eigen::Map<const Vector6> v(est);
    _estimate = internal::fromVectorMQT(v);
    return true;
  }

  virtual bool getMinimalEstimateData(number_t* est) const {
    Eigen::Map<Vector6> v(est);
    v = internal::toVectorMQT(_estimate);
    return true;
  }

  virtual int minimalEstimateDimension() const { return 6; }

  virtual void oplusImpl(const number_t* update) {
    Eigen::Map<const Vector6> v(update);
    Isometry3 increment = internal::fromVectorMQT(v);
    _estimate = _estimate * increment;
    if (++_numOplusCalls > orthogonalizeAfter) {
      _numOplusCalls = 0;
      internal::approximateNearestOrthogonalMatrix(
          _estimate.matrix().topLeftCorner<3, 3>());
    }
  }

  virtual bool read(std::istream& is) {
    Vector7 est;
    bool state = internal::readVector(is, est);
    setEstimate(internal::fromVectorQT(est));
    return state;
  }
  virtual bool write(std::ostream& os) const {
    return internal::writeVector(os, internal::toVectorQT(estimate()));
  }

 protected:
  int _numOplusCalls;  ///< store how often opluse was called to trigger
                       ///< orthogonaliation of the rotation matrix
};

}  // namespace g2o

#endif
