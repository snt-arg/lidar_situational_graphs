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

#ifndef EDGE_CORRIDOR_PLANE_HPP
#define EDGE_CORRIDOR_PLANE_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "g2o/vertex_corridor.hpp"
#include <g2o/core/base_multi_edge.h>

namespace g2o {

class EdgeSE3Corridor : public BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexCorridor> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Corridor() : BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexCorridor>() {
    _information.setIdentity();
  }

  void computeError() override {
    const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexCorridor* v2 = static_cast<const VertexCorridor*>(_vertices[1]);
    Eigen::Isometry3d w2m = v1->estimate().inverse();
    Eigen::Isometry3d corridor_pose_map;
    corridor_pose_map.matrix().block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    corridor_pose_map.matrix()(1, 3) = v2->estimate();

    Eigen::Isometry3d corridor_pose_local = corridor_pose_map * w2m;
    double est = corridor_pose_local.matrix()(0, 3);

    _error[0] = est - _measurement;
  }

  virtual bool read(std::istream& is) override {
    double v;
    is >> v;

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
    os << _measurement << " ";

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const double& m) override {
    _measurement = m;
  }
};

class EdgeCorridorXPlane : public BaseBinaryEdge<1, double, g2o::VertexCorridor, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeCorridorXPlane() : BaseBinaryEdge<1, double, g2o::VertexCorridor, g2o::VertexPlane>() {}

  void computeError() override {
    const VertexCorridor* v1 = static_cast<const VertexCorridor*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    double trans = v1->estimate();
    Eigen::Vector4d plane = v2->estimate().coeffs();

    plane(3) = -1 * plane(3);
    double p_norm = plane(0) / fabs(plane(0));
    plane(3) = p_norm * plane(3);

    double est;
    if(fabs(trans) > fabs(plane(3))) {
      est = trans - plane(3);
    } else {
      est = plane(3) - trans;
    }

    _error[0] = est - _measurement;
  }

  virtual bool read(std::istream& is) override {
    double v;
    is >> v;

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
    os << _measurement << " ";

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual int measurementDimension() const override {
    return 1;
  }
};

class EdgeCorridorYPlane : public BaseBinaryEdge<1, double, g2o::VertexCorridor, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeCorridorYPlane() : BaseBinaryEdge<1, double, g2o::VertexCorridor, g2o::VertexPlane>() {}

  void computeError() override {
    const VertexCorridor* v1 = static_cast<const VertexCorridor*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    double trans = v1->estimate();
    Eigen::Vector4d plane = v2->estimate().coeffs();

    plane(3) = -1 * plane(3);
    double p_norm = plane(1) / fabs(plane(1));
    plane(3) = p_norm * plane(3);

    double est;
    if(fabs(trans) > fabs(plane(3))) {
      est = trans - plane(3);
    } else {
      est = plane(3) - trans;
    }

    _error[0] = est - _measurement;
  }

  virtual bool read(std::istream& is) override {
    double v;
    is >> v;

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
    os << _measurement << " ";

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

  virtual void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual int measurementDimension() const override {
    return 1;
  }
};

class EdgeRoom2Planes : public BaseMultiEdge<2, Eigen::Vector2d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoom2Planes() : BaseMultiEdge<2, Eigen::Vector2d>() {
    resize(4);
  }

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);
    const VertexRoomXYLB* v4 = static_cast<const VertexRoomXYLB*>(_vertices[3]);

    Eigen::Vector2d room_pose = v1->estimate();
    Eigen::Vector4d plane1 = v2->estimate().coeffs();
    Eigen::Vector4d plane2 = v3->estimate().coeffs();
    Eigen::Vector2d cluster_center = v4->estimate();

    correct_plane_direction(plane1);
    correct_plane_direction(plane2);

    Eigen::Vector3d vec;
    if(fabs(plane1(3)) > fabs(plane2(3))) {
      vec = (0.5 * (fabs(plane1(3)) * plane1.head(3) - fabs(plane2(3)) * plane2.head(3))) + fabs(plane2(3)) * plane2.head(3);
    } else {
      vec = (0.5 * (fabs(plane2(3)) * plane2.head(3) - fabs(plane1(3)) * plane1.head(3))) + fabs(plane1(3)) * plane1.head(3);
    }

    Eigen::Vector2d vec_normal = vec.head(2) / vec.head(2).norm();
    Eigen::Vector2d final_pose_vec = vec.head(2) + (cluster_center - (cluster_center.dot(vec_normal)) * vec_normal);

    _error = room_pose - final_pose_vec;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector2d v;
    is >> v(0) >> v(1);

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
    Eigen::Vector2d v = _measurement;
    os << v(0) << " " << v(1) << " ";

    for(int i = 0; i < information().rows(); ++i) {
      for(int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      };
    }
    return os.good();
  }

private:
  void correct_plane_direction(Eigen::Vector4d& plane) {
    if(plane(3) > 0) {
      plane(0) = -1 * plane(0);
      plane(1) = -1 * plane(1);
      plane(2) = -1 * plane(2);
      plane(3) = -1 * plane(3);
    } else {
      plane = plane;
    }
  }
};

}  // namespace g2o
#endif  // EDGE_CORRIDOR_PLANE_HPP
