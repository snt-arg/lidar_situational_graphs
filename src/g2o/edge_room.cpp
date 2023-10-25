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
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_room.hpp>

#include "g2o/types/slam3d/isometry3d_gradients.h"
#include "g2o/vertex_floor.hpp"
#include "g2o/vertex_infinite_room.hpp"
#include "g2o/vertex_room.hpp"

namespace g2o {

EdgeRoomRoom::EdgeRoomRoom() : BaseBinaryEdge<6, Isometry3, VertexRoom, VertexRoom>() {
  information().setIdentity();
}

bool EdgeRoomRoom::read(std::istream& is) {
  Vector7 meas;
  internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurement(internal::fromVectorQT(meas));
  if (is.bad()) return false;
  readInformationMatrix(is);
  return is.good() || is.eof();
}

bool EdgeRoomRoom::write(std::ostream& os) const {
  internal::writeVector(os, internal::toVectorQT(measurement()));
  return writeInformationMatrix(os);
}

void EdgeRoomRoom::computeError() {
  VertexRoom* from = static_cast<VertexRoom*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);
  Isometry3 delta = _inverseMeasurement * from->estimate().inverse() * to->estimate();
  _error = internal::toVectorMQT(delta);
}

bool EdgeRoomRoom::setMeasurementFromState() {
  VertexRoom* from = static_cast<VertexRoom*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);
  Isometry3 delta = from->estimate().inverse() * to->estimate();
  setMeasurement(delta);
  return true;
}

void EdgeRoomRoom::linearizeOplus() {
  // BaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>::linearizeOplus();
  // return;

  VertexRoom* from = static_cast<VertexRoom*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);
  Isometry3 E;
  const Isometry3& Xi = from->estimate();
  const Isometry3& Xj = to->estimate();
  const Isometry3& Z = _measurement;
  internal::computeEdgeSE3Gradient(E, _jacobianOplusXi, _jacobianOplusXj, Z, Xi, Xj);
}

void EdgeRoomRoom::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                   OptimizableGraph::Vertex* /*to_*/) {
  VertexRoom* from = static_cast<VertexRoom*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);

  if (from_.count(from) > 0) {
    to->setEstimate(from->estimate() * _measurement);
  } else
    from->setEstimate(to->estimate() * _measurement.inverse());
  // cerr << "IE" << endl;
}

void EdgeSE3Room::computeError() {
  VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);
  Isometry3 delta = _inverseMeasurement * from->estimate().inverse() * to->estimate();
  _error = internal::toVectorMQT(delta);
}

void EdgeSE3Room::linearizeOplus() {
  // BaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>::linearizeOplus();
  // return;

  VertexSE3* from = static_cast<VertexSE3*>(_vertices[0]);
  VertexRoom* to = static_cast<VertexRoom*>(_vertices[1]);
  Isometry3 E;
  const Isometry3& Xi = from->estimate();
  const Isometry3& Xj = to->estimate();
  const Isometry3& Z = _measurement;
  internal::computeEdgeSE3Gradient(E, _jacobianOplusXi, _jacobianOplusXj, Z, Xi, Xj);
}
bool EdgeSE3Room::read(std::istream& is) {
  Vector7 meas;
  internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurement(internal::fromVectorQT(meas));
  if (is.bad()) return false;
  readInformationMatrix(is);
  return is.good() || is.eof();
  Eigen::Vector2d v;
  is >> v(0) >> v(1);
}

bool EdgeSE3Room::write(std::ostream& os) const {
  internal::writeVector(os, internal::toVectorQT(measurement()));
  return writeInformationMatrix(os);
}

void EdgeRoom2Planes::computeError() {
  const VertexRoom* v1 = static_cast<const VertexRoom*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);
  const VertexRoom* v4 = static_cast<const VertexRoom*>(_vertices[3]);

  Eigen::Vector2d room_pose = v1->estimate().translation().head(2);
  Eigen::Vector4d plane1 = v2->estimate().coeffs();
  Eigen::Vector4d plane2 = v3->estimate().coeffs();
  Eigen::Vector2d cluster_center = v4->estimate().translation().head(2);

  correct_plane_direction(plane1);
  correct_plane_direction(plane2);

  Eigen::Vector3d vec;
  if (fabs(plane1(3)) > fabs(plane2(3))) {
    vec =
        (0.5 * (fabs(plane1(3)) * plane1.head(3) - fabs(plane2(3)) * plane2.head(3))) +
        fabs(plane2(3)) * plane2.head(3);
  } else {
    vec =
        (0.5 * (fabs(plane2(3)) * plane2.head(3) - fabs(plane1(3)) * plane1.head(3))) +
        fabs(plane1(3)) * plane1.head(3);
  }

  Eigen::Vector2d vec_normal = vec.head(2) / vec.head(2).norm();
  Eigen::Vector2d final_pose_vec =
      vec.head(2) + (cluster_center - (cluster_center.dot(vec_normal)) * vec_normal);

  _error = room_pose - final_pose_vec;
}

bool EdgeRoom2Planes::read(std::istream& is) {
  Eigen::Vector2d v;
  is >> v(0) >> v(1);

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

bool EdgeRoom2Planes::write(std::ostream& os) const {
  Eigen::Vector2d v = _measurement;
  os << v(0) << " " << v(1) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeRoom2Planes::correct_plane_direction(Eigen::Vector4d& plane) {
  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  } else {
    plane = plane;
  }
}

void EdgeRoom4Planes::computeError() {
  const VertexRoom* v1 = static_cast<const VertexRoom*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);
  const VertexPlane* v4 = static_cast<const VertexPlane*>(_vertices[3]);
  const VertexPlane* v5 = static_cast<const VertexPlane*>(_vertices[4]);

  Eigen::Vector2d room_pose = v1->estimate().translation().head(2);
  Eigen::Vector4d x_plane1 = v2->estimate().coeffs();
  Eigen::Vector4d x_plane2 = v3->estimate().coeffs();
  Eigen::Vector4d y_plane1 = v4->estimate().coeffs();
  Eigen::Vector4d y_plane2 = v5->estimate().coeffs();

  correct_plane_direction(x_plane1);
  correct_plane_direction(x_plane2);
  correct_plane_direction(y_plane1);
  correct_plane_direction(y_plane2);

  Eigen::Vector3d vec_x, vec_y;
  if (fabs(x_plane1(3)) > fabs(x_plane2(3))) {
    vec_x = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) -
                    fabs(x_plane2(3)) * x_plane2.head(3))) +
            fabs(x_plane2(3)) * x_plane2.head(3);
  } else {
    vec_x = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) -
                    fabs(x_plane1(3)) * x_plane1.head(3))) +
            fabs(x_plane1(3)) * x_plane1.head(3);
  }

  if (fabs(y_plane1(3)) > fabs(y_plane2(3))) {
    vec_y = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) -
                    fabs(y_plane2(3)) * y_plane2.head(3))) +
            fabs(y_plane2(3)) * y_plane2.head(3);
  } else {
    vec_y = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) -
                    fabs(y_plane1(3)) * y_plane1.head(3))) +
            fabs(y_plane1(3)) * y_plane1.head(3);
  }

  Eigen::Vector2d final_vec = vec_x.head(2) + vec_y.head(2);
  _error = room_pose - final_vec;
}

bool EdgeRoom4Planes::read(std::istream& is) {
  Eigen::Vector2d v;
  is >> v(0) >> v(1);

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

bool EdgeRoom4Planes::write(std::ostream& os) const {
  Eigen::Vector2d v = _measurement;
  os << v(0) << " " << v(1) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeRoom4Planes::correct_plane_direction(Eigen::Vector4d& plane) {
  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  } else {
    plane = plane;
  }
}

void EdgeFloorRoom::computeError() {
  const VertexFloor* v1 = static_cast<const VertexFloor*>(_vertices[0]);
  const VertexRoom* v2 = static_cast<const VertexRoom*>(_vertices[1]);

  Eigen::Vector2d est;
  est(0) = v1->estimate().translation()(0) - v2->estimate().translation()(0);
  est(1) = v1->estimate().translation()(1) - v2->estimate().translation()(1);

  _error = est - _measurement;
}

bool EdgeFloorRoom::read(std::istream& is) {
  Eigen::Vector2d v;
  is >> v(0) >> v(1);

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

bool EdgeFloorRoom::write(std::ostream& os) const {
  Eigen::Vector2d v = _measurement;
  os << v(0) << " " << v(1) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void Edge2Rooms::computeError() {
  const VertexRoom* v1 = static_cast<const VertexRoom*>(_vertices[0]);
  const VertexRoom* v2 = static_cast<const VertexRoom*>(_vertices[1]);
  Eigen::Vector2d v1_est, v2_est;
  v1_est(0) = v1->estimate().translation().x();
  v1_est(1) = v1->estimate().translation().y();
  v2_est(0) = v2->estimate().translation().x();
  v2_est(1) = v2->estimate().translation().y();

  _error = v1_est - v2_est;
}

bool Edge2Rooms::read(std::istream& is) {
  Eigen::Vector2d v;
  is >> v(0) >> v(1);

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

bool Edge2Rooms::write(std::ostream& os) const {
  Eigen::Vector2d v;
  os << v(0) << " " << v(1) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

}  // namespace g2o
