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

#ifndef EDGE_ROOM_PLANE_HPP
#define EDGE_ROOM_PLANE_HPP

#include <Eigen/Dense>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include "g2o/vertex_room.hpp"
#include "g2o/vertex_corridor.hpp"
namespace g2o {

class EdgeSE3Room : public BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE3, g2o::VertexRoomXYLB> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Room() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE3, g2o::VertexRoomXYLB>() {}

  void computeError() override {
    const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
    const VertexRoomXYLB* v2 = static_cast<const VertexRoomXYLB*>(_vertices[1]);

    Eigen::Isometry3d m2l = v1->estimate().inverse();
    Eigen::Isometry3d room_map;
    room_map.matrix().block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    room_map.matrix().block<2, 1>(0, 3) = v2->estimate();

    Eigen::Isometry3d room_local = room_map * m2l;
    Eigen::Vector2d est = room_local.matrix().block<2, 1>(0, 3);
    _error = est - _measurement;
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

  virtual void setMeasurement(const Eigen::Vector2d& m) override {
    _measurement = m;
  }
};
class EdgeRoomXPlane : public BaseBinaryEdge<1, double, VertexRoomXYLB, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomXPlane() : BaseBinaryEdge<1, double, VertexRoomXYLB, g2o::VertexPlane>() {}

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    Eigen::Vector2d room_pose = v1->estimate();
    Eigen::Vector4d plane = v2->estimate().coeffs();

    if(plane(3) > 0) {
      plane(0) = -1 * plane(0);
      plane(1) = -1 * plane(1);
      plane(2) = -1 * plane(2);
      plane(3) = -1 * plane(3);
    }

    double est;
    if(fabs(room_pose(0)) > fabs(plane(3))) {
      est = room_pose(0) - plane(3);
    } else {
      est = plane(3) - room_pose(0);
    }

    if(est * _measurement < 0) {
      est = -1 * est;
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

class EdgeRoomYPlane : public BaseBinaryEdge<1, double, VertexRoomXYLB, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomYPlane() : BaseBinaryEdge<1, double, VertexRoomXYLB, g2o::VertexPlane>() {}

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    Eigen::Vector2d room_pose = v1->estimate();
    Eigen::Vector4d plane = v2->estimate().coeffs();

    if(plane(3) > 0) {
      plane(0) = -1 * plane(0);
      plane(1) = -1 * plane(1);
      plane(2) = -1 * plane(2);
      plane(3) = -1 * plane(3);
    }

    double est;
    if(fabs(room_pose(1)) > fabs(plane(3))) {
      est = room_pose(1) - plane(3);
    } else {
      est = plane(3) - room_pose(1);
    }

    if(est * _measurement < 0) {
      est = -1 * est;
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

    correct_plane_d(plane1);
    correct_plane_d(plane2);

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
  void correct_plane_d(Eigen::Vector4d& plane) {
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

class EdgeRoom4Planes : public BaseMultiEdge<2, Eigen::Vector2d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoom4Planes() : BaseMultiEdge<2, Eigen::Vector2d>() {
    resize(5);
  }
  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
    const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);
    const VertexPlane* v4 = static_cast<const VertexPlane*>(_vertices[3]);
    const VertexPlane* v5 = static_cast<const VertexPlane*>(_vertices[4]);

    Eigen::Vector2d room_pose = v1->estimate();
    Eigen::Vector4d x_plane1 = v2->estimate().coeffs();
    Eigen::Vector4d x_plane2 = v3->estimate().coeffs();
    Eigen::Vector4d y_plane1 = v4->estimate().coeffs();
    Eigen::Vector4d y_plane2 = v5->estimate().coeffs();

    correct_plane_d(x_plane1);
    correct_plane_d(x_plane2);
    correct_plane_d(y_plane1);
    correct_plane_d(y_plane2);

    Eigen::Vector3d vec_x, vec_y;
    if(fabs(x_plane1(3)) > fabs(x_plane2(3))) {
      vec_x = (0.5 * (fabs(x_plane1(3)) * x_plane1.head(3) - fabs(x_plane2(3)) * x_plane2.head(3))) + fabs(x_plane2(3)) * x_plane2.head(3);
    } else {
      vec_x = (0.5 * (fabs(x_plane2(3)) * x_plane2.head(3) - fabs(x_plane1(3)) * x_plane1.head(3))) + fabs(x_plane1(3)) * x_plane1.head(3);
    }

    if(fabs(y_plane1(3)) > fabs(y_plane2(3))) {
      vec_y = (0.5 * (fabs(y_plane1(3)) * y_plane1.head(3) - fabs(y_plane2(3)) * y_plane2.head(3))) + fabs(y_plane2(3)) * y_plane2.head(3);
    } else {
      vec_y = (0.5 * (fabs(y_plane2(3)) * y_plane2.head(3) - fabs(y_plane1(3)) * y_plane1.head(3))) + fabs(y_plane1(3)) * y_plane1.head(3);
    }

    Eigen::Vector2d final_vec = vec_x.head(2) + vec_y.head(2);
    _error = room_pose - final_vec;
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
  void correct_plane_d(Eigen::Vector4d& plane) {
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

class EdgeRoomRoom : public BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexRoomXYLB, g2o::VertexRoomXYLB> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomRoom() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexRoomXYLB, g2o::VertexRoomXYLB>() {}

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexRoomXYLB* v2 = static_cast<const VertexRoomXYLB*>(_vertices[1]);

    Eigen::Vector2d est;
    est(0) = v1->estimate()(0) - v2->estimate()(0);
    est(1) = v1->estimate()(1) - v2->estimate()(1);

    _error = est - _measurement;
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

  virtual void setMeasurement(const Eigen::Vector2d& m) override {
    _measurement = m;
  }
};

class EdgeRoomXCorridor : public BaseBinaryEdge<1, double, g2o::VertexRoomXYLB, g2o::VertexCorridor> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomXCorridor() : BaseBinaryEdge<1, double, g2o::VertexRoomXYLB, g2o::VertexCorridor>() {}

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexCorridor* v2 = static_cast<const g2o::VertexCorridor*>(_vertices[1]);

    double est;
    est = pow(v1->estimate()(0) - v2->estimate(), 2);

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

class EdgeRoomYCorridor : public BaseBinaryEdge<1, double, g2o::VertexRoomXYLB, g2o::VertexCorridor> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomYCorridor() : BaseBinaryEdge<1, double, g2o::VertexRoomXYLB, g2o::VertexCorridor>() {}

  void computeError() override {
    const VertexRoomXYLB* v1 = static_cast<const VertexRoomXYLB*>(_vertices[0]);
    const VertexCorridor* v2 = static_cast<const g2o::VertexCorridor*>(_vertices[1]);

    double est;
    est = pow(v1->estimate()(1) - v2->estimate(), 2);

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

class EdgeRoomXPrior : public g2o::BaseUnaryEdge<1, double, g2o::VertexRoomXYLB> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomXPrior() : g2o::BaseUnaryEdge<1, double, g2o::VertexRoomXYLB>() {}

  void computeError() override {
    const g2o::VertexRoomXYLB* v1 = static_cast<const g2o::VertexRoomXYLB*>(_vertices[0]);

    Eigen::Vector2d estimate = v1->estimate();

    _error[0] = estimate(0) - _measurement;
  }

  void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    double v;
    is >> v;
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }

  virtual bool write(std::ostream& os) const override {
    os << _measurement << " ";

    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};

class EdgeRoomYPrior : public g2o::BaseUnaryEdge<1, double, g2o::VertexRoomXYLB> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeRoomYPrior() : g2o::BaseUnaryEdge<1, double, g2o::VertexRoomXYLB>() {}

  void computeError() override {
    const g2o::VertexRoomXYLB* v1 = static_cast<const g2o::VertexRoomXYLB*>(_vertices[0]);

    Eigen::Vector2d estimate = v1->estimate();

    _error[0] = estimate(1) - _measurement;
  }

  void setMeasurement(const double& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    double v;
    is >> v;
    setMeasurement(v);

    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }

  virtual bool write(std::ostream& os) const override {
    os << _measurement << " ";

    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};

}  // namespace g2o
#endif  // EDGE_ROOM_PLANE_HPP
