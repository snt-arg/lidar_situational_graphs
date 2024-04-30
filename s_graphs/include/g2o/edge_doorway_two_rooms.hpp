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

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_doorway_two_rooms.hpp>
#include <g2o/vertex_doorway.hpp>

#include "g2o/vertex_infinite_room.hpp"
#include "g2o/vertex_room.hpp"

namespace g2o {

/*   Define Doorway edge with 2 rooms here*/

class EdgeDoorWay2Rooms : public BaseMultiEdge<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeDoorWay2Rooms() : BaseMultiEdge<3, Eigen::Vector3d>() { resize(4); }

  void computeError() override {
    const VertexDoorWay* v1 = static_cast<const VertexDoorWay*>(_vertices[0]);
    const VertexDoorWay* v2 = static_cast<const VertexDoorWay*>(_vertices[1]);
    const VertexRoom* v3 = static_cast<const VertexRoom*>(_vertices[2]);
    const VertexRoom* v4 = static_cast<const VertexRoom*>(_vertices[3]);

    Eigen::Isometry3d doorway1_coord_r1 = v1->estimate();
    Eigen::Isometry3d doorway1_coord_r2 = v2->estimate();
    Eigen::Isometry3d room1_coord_w = v3->estimate();
    Eigen::Isometry3d room2_coord_w = v4->estimate();

    // Convert room 1 center to room pose (Transformation matrix)
    Eigen::Matrix4d room1_pose_w;
    Eigen::Vector3d room1_trans;
    room1_pose_w.setIdentity();
    room1_trans.x() = room1_coord_w.translation().x();
    room1_trans.y() = room1_coord_w.translation().y();
    room1_trans.z() = room1_coord_w.translation().z();
    room1_pose_w.topRightCorner<3, 1>() = room1_trans;

    // Convert door1 position vector to 4x1
    Eigen::Vector4d doorway1_pose_r1(doorway1_coord_r1.translation().x(),
                                     doorway1_coord_r1.translation().y(),
                                     doorway1_coord_r1.translation().z(),
                                     1);

    // Convert room 2 center to room pose (Transformation matrix)
    Eigen::Matrix4d room2_pose_w;
    Eigen::Vector3d room2_trans;
    room2_pose_w.setIdentity();
    room2_trans.x() = room2_coord_w.translation().x();
    room2_trans.y() = room2_coord_w.translation().y();
    room2_trans.z() = room2_coord_w.translation().z();
    room2_pose_w.topRightCorner<3, 1>() = room2_trans;

    // Convert door1 position vector to 4x1
    Eigen::Vector4d doorway1_pose_r2(doorway1_coord_r2.translation().x(),
                                     doorway1_coord_r2.translation().y(),
                                     doorway1_coord_r2.translation().z(),
                                     1);

    // Calculate error
    Eigen::Vector4d diff =
        (room1_pose_w * doorway1_pose_r1) - (room2_pose_w * doorway1_pose_r2);
    Eigen::Vector3d error(diff.x(), diff.y(), diff.z());
    _error = error;
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
};

}  // namespace g2o
#endif  // EDGE_DOORWAY_2_ROOMS_HPP
