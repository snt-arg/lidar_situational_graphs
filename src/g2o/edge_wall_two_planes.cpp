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

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_wall_two_planes.hpp>

#include "g2o/vertex_wall.hpp"
namespace g2o {

/*   Define Wall edge with wall surfaces here*/

void EdgeWall2Planes::computeError() {
  const VertexWallXYZ* v1 = static_cast<const VertexWallXYZ*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);

  Eigen::Vector3d wall_center = v1->estimate();
  Eigen::Vector4d plane1 = v2->estimate().coeffs();
  Eigen::Vector4d plane2 = v3->estimate().coeffs();

  correct_plane_direction(plane1);
  correct_plane_direction(plane2);

  Eigen::Vector3d estimated_wall_center;
  if (_use_factor_nn){
    estimated_wall_center = compute_factor_nn(plane1, plane2);
  } else {
    estimated_wall_center = compute_factor_legacy(plane1, plane2);
  }

  _error = wall_center - estimated_wall_center;
}

bool EdgeWall2Planes::read(std::istream& is) {
  Eigen::Vector3d v;
  is >> v(0) >> v(1) >> v(2);

  setMeasurement(v);
  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }

  return true;
}

bool EdgeWall2Planes::write(std::ostream& os) const {
  Eigen::Vector3d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeWall2Planes::correct_plane_direction(Eigen::Vector4d& plane) {
  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  } else {
    plane = plane;
  }
}

Eigen::Vector3d EdgeWall2Planes::compute_factor_legacy(Eigen::Vector4d plane1, Eigen::Vector4d plane2) {
  Eigen::Vector3d estimated_wall_center;
  if (fabs(plane1(3)) > fabs(plane2(3))) {
    estimated_wall_center =
        (0.5 * (fabs(plane1(3)) * plane1.head(3) - fabs(plane2(3)) * plane2.head(3))) +
        fabs(plane2(3)) * plane2.head(3);
  } else {
    estimated_wall_center =
        (0.5 * (fabs(plane2(3)) * plane2.head(3) - fabs(plane1(3)) * plane1.head(3))) +
        fabs(plane1(3)) * plane1.head(3);
  }

  Eigen::Vector3d estimated_wall_center_normalized =
      estimated_wall_center.head(2) / estimated_wall_center.norm();
  Eigen::Vector3d final_wall_center =
      estimated_wall_center.head(2) +
      (_wall_point - (_wall_point.dot(estimated_wall_center_normalized)) *
                         estimated_wall_center_normalized);

  return final_wall_center;
}

Eigen::Vector3d EdgeWall2Planes::compute_factor_nn(Eigen::Vector4d plane1, Eigen::Vector4d plane2) {
  float normalization = 20.0;
  std::vector<Eigen::Vector4d> vectorList;
    vectorList.push_back(plane1);
    vectorList.push_back(plane2);

  std::vector<float> concatenatedVector;
  for (const Eigen::Vector4d& vector : vectorList) {
      concatenatedVector.push_back(static_cast<float>(vector(0)));
      concatenatedVector.push_back(static_cast<float>(vector(1)));
      concatenatedVector.push_back(static_cast<float>(vector(2)));
      concatenatedVector.push_back(static_cast<float>(vector(3))/normalization);
  }

  std::cout << "FLAG concatenatedVector " << concatenatedVector << '\n';
  std::vector<std::vector<float>> allNeighborsVector; /// TODO change
  Eigen::Vector2d output = factor_nn.infer(allNeighborsVector);
  Eigen::Vector3d final_output;
  std::cout << "FLAG output " << output << '\n';
  final_output[0] = output[0] * normalization;
  final_output[1] = output[1] * normalization;
  final_output[2] = 0.0;
  return final_output;
}

}  // namespace g2o
