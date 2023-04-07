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
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_infinite_room_plane.hpp>

namespace g2o {

void EdgeSE3InfiniteRoom::computeError() {
  const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
  const VertexInfiniteRoom* v2 = static_cast<const VertexInfiniteRoom*>(_vertices[1]);
  Eigen::Isometry3d w2m = v1->estimate().inverse();
  Eigen::Isometry3d infinite_room_pose_map;
  infinite_room_pose_map.matrix().block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
  infinite_room_pose_map.matrix()(1, 3) = v2->estimate();

  Eigen::Isometry3d infinite_room_pose_local = infinite_room_pose_map * w2m;
  double est = infinite_room_pose_local.matrix()(0, 3);

  _error[0] = est - _measurement;
}

bool EdgeSE3InfiniteRoom::read(std::istream& is) {
  double v;
  is >> v;

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

bool EdgeSE3InfiniteRoom::write(std::ostream& os) const {
  os << _measurement << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeInfiniteRoomXPlane::computeError() {
  const VertexInfiniteRoom* v1 = static_cast<const VertexInfiniteRoom*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  double trans = v1->estimate();
  Eigen::Vector4d plane = v2->estimate().coeffs();

  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  }

  double est;
  if (fabs(trans) > fabs(plane(3))) {
    est = trans - plane(3);
  } else {
    est = plane(3) - trans;
  }

  if (est * _measurement < 0) {
    est = -1 * est;
  }

  _error[0] = est - _measurement;
}

bool EdgeInfiniteRoomXPlane::read(std::istream& is) {
  double v;
  is >> v;

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

bool EdgeInfiniteRoomXPlane::write(std::ostream& os) const {
  os << _measurement << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeInfiniteRoomYPlane::computeError() {
  const VertexInfiniteRoom* v1 = static_cast<const VertexInfiniteRoom*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  double trans = v1->estimate();
  Eigen::Vector4d plane = v2->estimate().coeffs();

  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  }

  double est;
  if (fabs(trans) > fabs(plane(3))) {
    est = trans - plane(3);
  } else {
    est = plane(3) - trans;
  }

  if (est * _measurement < 0) {
    est = -1 * est;
  }

  _error[0] = est - _measurement;
}

bool EdgeInfiniteRoomYPlane::read(std::istream& is) {
  double v;
  is >> v;

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

bool EdgeInfiniteRoomYPlane::write(std::ostream& os) const {
  os << _measurement << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeXInfiniteRoomXInfiniteRoom::computeError() {
  const VertexInfiniteRoom* v1 = static_cast<const VertexInfiniteRoom*>(_vertices[0]);
  const VertexInfiniteRoom* v2 = static_cast<const VertexInfiniteRoom*>(_vertices[1]);

  double est = pow(v1 - v2, 2);

  _error[0] = est - _measurement;
}

bool EdgeXInfiniteRoomXInfiniteRoom::read(std::istream& is) {
  double v;
  is >> v;

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

bool EdgeXInfiniteRoomXInfiniteRoom::write(std::ostream& os) const {
  os << _measurement << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeYInfiniteRoomYInfiniteRoom::computeError() {
  const VertexInfiniteRoom* v1 = static_cast<const VertexInfiniteRoom*>(_vertices[0]);
  const VertexInfiniteRoom* v2 = static_cast<const VertexInfiniteRoom*>(_vertices[1]);

  double est = pow(v1 - v2, 2);

  _error[0] = est - _measurement;
}

bool EdgeYInfiniteRoomYInfiniteRoom::read(std::istream& is) {
  double v;
  is >> v;

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

bool EdgeYInfiniteRoomYInfiniteRoom::write(std::ostream& os) const {
  os << _measurement << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

}  // namespace g2o
