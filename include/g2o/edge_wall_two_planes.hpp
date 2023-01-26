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

#ifndef EDGE_WALL_2_PLANES_HPP
#define EDGE_WALL_2_PLANES_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>

#include "g2o/vertex_wall.hpp"
namespace g2o {

/*   Define Wall edge with wall surfaces here*/

class EdgeWall2Planes : public BaseMultiEdge<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeWall2Planes(Eigen::Vector3d wall_point) : BaseMultiEdge<3, Eigen::Vector3d>() {
    resize(3);
    _wall_point = wall_point;
  }

  void computeError() override {
    const VertexWallXYZ* v1 = static_cast<const VertexWallXYZ*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);

    Eigen::Vector3d wall_center = v1->estimate();
    Eigen::Vector4d plane1 = v2->estimate().coeffs();
    Eigen::Vector4d plane2 = v3->estimate().coeffs();

    correct_plane_direction(plane1);
    correct_plane_direction(plane2);

    Eigen::Vector3d estimated_wall_center;
    if (fabs(plane1(3)) > fabs(plane2(3))) {
      estimated_wall_center = (0.5 * (fabs(plane1(3)) * plane1.head(3) -
                                      fabs(plane2(3)) * plane2.head(3))) +
                              fabs(plane2(3)) * plane2.head(3);
    } else {
      estimated_wall_center = (0.5 * (fabs(plane2(3)) * plane2.head(3) -
                                      fabs(plane1(3)) * plane1.head(3))) +
                              fabs(plane1(3)) * plane1.head(3);
    }

    Eigen::Vector3d estimated_wall_center_normalized =
        estimated_wall_center.head(2) / estimated_wall_center.norm();
    Eigen::Vector3d final_wall_center =
        estimated_wall_center.head(2) +
        (_wall_point - (_wall_point.dot(estimated_wall_center_normalized)) *
                           estimated_wall_center_normalized);

    _error = wall_center - final_wall_center;
  }

  virtual bool read(std::istream& is) override {
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

  virtual bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";

    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

 private:
  void correct_plane_direction(Eigen::Vector4d& plane) {
    if (plane(3) > 0) {
      plane(0) = -1 * plane(0);
      plane(1) = -1 * plane(1);
      plane(2) = -1 * plane(2);
      plane(3) = -1 * plane(3);
    } else {
      plane = plane;
    }
  }
  Eigen::Vector3d _wall_point;
};

}  // namespace g2o
#endif  // EDGE_WALL_2_PLANES_HPP
