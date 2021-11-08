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

#ifndef KKL_G2O_EDGE_SE3_POINT_TO_PLANE_HPP
#define KKL_G2O_EDGE_SE3_POINT_TO_PLANE_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {
class EdgeSE3PointToPlane : public g2o::BaseBinaryEdge<1, Eigen::Matrix4d, g2o::VertexSE3, g2o::VertexPlane> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PointToPlane() : BaseBinaryEdge<1, Eigen::Matrix4d, g2o::VertexSE3, g2o::VertexPlane>() {}

  void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
    const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);
    
    Eigen::Matrix4d Ti =  v1->estimate().matrix();
    Eigen::Vector4d Pj =  v2->estimate().toVector();
    // Eigen::Vector3d plane_point = Pj.head(2) * Pj(3); 
    // Pj.head(2) = plane_point / plane_point.norm(); 
    // Pj(3) = plane_point.norm();
    Eigen::Matrix4d Gij = _measurement;
    _error = Pj.transpose() * Ti * Gij * Ti.transpose() * Pj / 2;
    //std::cout << "error " << _error << std::endl;
  }

  void setMeasurement(const Eigen::Matrix4d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Matrix4d v;
    for(int i = 0; i < measurement().rows(); ++i)
      for(int j = 0; j < measurement().cols(); ++j) {
        is >> v(i,j);
    }
    setMeasurement(Eigen::Matrix4d(v));

    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    Eigen::Matrix4d v = _measurement;
    
    for(int i = 0; i < measurement().rows(); ++i)
      for(int j = 0; j < measurement().cols(); ++j) {
        os << " " << v(i,j);
      }

    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }

};
}  // namespace g2o

#endif