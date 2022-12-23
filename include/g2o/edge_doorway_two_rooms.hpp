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

#ifndef EDGE_DOORWAY_2_ROOMS_HPP
#define EDGE_DOORWAY_2_ROOMS_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "g2o/vertex_wall.hpp"
namespace g2o {

/*   Define Wall edge with wall surfaces here*/

class EdgeDoor2Rooms : public BaseMultiEdge<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeDoorWay2Rooms() : BaseMultiEdge<4, Eigen::Vector4d>() {
    resize(4);
  }

  void computeError() override {
    const VertexDoorWayXYZ* v1 = static_cast<const VertexDoorWayXYZ*>(_vertices[0]);
    const VertexDoorWayXYZ* v2 = static_cast<const VertexDoorWayXYZ*>(_vertices[1]);
    const VertexRoomXYLB* v3 = static_cast<const VertexRoomXYLB*>(_vertices[2]);
    const VertexRoomXYLB* v4 = static_cast<const VertexRoomXYLB*>(_vertices[3]);

    Eigen::Vector3d door_way1 = v1->estimate();
    Eigen::Vector3d door_way2 = v2->estimate();
    Eigen::Vector2d room1 = v3->estimate();
    Eigen::Vector2d room2 = v4->estimate();

    _error = wall_center - final_wall_center;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector4d v;
    is >> v(0) >> v(1) >> v(2) >> v(3);

    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }

    return true;
  }

  virtual bool write(std::ostream& os) const override {
    Eigen::Vector4d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " " v(3) << " ";

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  Eigen::Vector3d _wall_point;
};

}  // namespace g2o
#endif  // EDGE_DOORWAY_2_ROOMS_HPP
