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

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <g2o/edge_se3_point_to_two_planes.hpp>

namespace g2o {

void EdgeSE3PlanePlane::computeError() {
  const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
  const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);
  const g2o::VertexPlane* v3 = static_cast<const g2o::VertexPlane*>(_vertices[2]);

  Eigen::Vector4d s_graph_plane = v2->estimate().toVector();
  Eigen::Vector4d a_graph_plane = v3->estimate().toVector();

  Eigen::Matrix4d deviation = v1->estimate().matrix();
  // Compute the translation vector as the difference between the normal vectors of the
  // planes
  // Eigen::Vector3d translation = a_graph_plane.normal() - s_graph_plane.normal();

  // double angleDifference = s_graph_plane.normal().angle(a_graph_plane.normal());
  // Eigen::Matrix3d rotation =
  //     Eigen::AngleAxisd(angleDifference,
  //     Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // v1->setEstimate(g2o::SE3Quat(rotation, translation));
  _error = deviation * a_graph_plane - s_graph_plane;
}

bool EdgeSE3PlanePlane::read(std::istream& is) {
  Eigen::Vector4d v;
  is >> v(0) >> v(1) >> v(2) >> v(3);
  setMeasurement(v);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgeSE3PlanePlane::write(std::ostream& os) const {
  Eigen::Vector4d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}

}  // namespace g2o
