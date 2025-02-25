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

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Dense>
#include <g2o/edge_multi_se3.hpp>

namespace g2o {

void EdgeMultiSE3::computeError() {
  const g2o::VertexSE3* a_graph_origin_node =
      static_cast<const g2o::VertexSE3*>(_vertices[0]);
  const g2o::VertexSE3* s_graph_origin_node =
      static_cast<const g2o::VertexSE3*>(_vertices[1]);
  const g2o::VertexSE3* transformation_node =
      static_cast<const g2o::VertexSE3*>(_vertices[2]);

  Isometry3 delta =
      (transformation_node->estimate() * s_graph_origin_node->estimate()) *
      a_graph_origin_node->estimate().inverse();

  _error = internal::toVectorMQT(delta);
}

bool EdgeMultiSE3::read(
    std::istream& is) {  // Read the measurement from the input stream
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
  for (int i = 0; i < 4; ++i) {
    is >> q.coeffs()[i];
  }
  for (int i = 0; i < 3; ++i) {
    is >> t[i];
  }

  // Set the measurement using the quaternion and translation
  _measurement.matrix() = Eigen::Matrix4d::Identity();
  _measurement.matrix().topLeftCorner(3, 3) = q.normalized().toRotationMatrix();
  _measurement.matrix().topRightCorner(3, 1) = t;

  // Read the information matrix (if needed)
  for (int i = 0; i < information().rows() && is.good(); ++i) {
    for (int j = i; j < information().cols() && is.good(); ++j) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }

  return true;
}
bool EdgeMultiSE3::write(
    std::ostream& os) const {  // Write the measurement (quaternion and translation)
  Eigen::Quaterniond q(_measurement.rotation());
  Eigen::Vector3d t(_measurement.translation());

  os << q.coeffs().transpose() << " " << t.transpose();

  // Write the information matrix (if needed)
  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    }
  }
  return os.good();
}
}  // namespace g2o