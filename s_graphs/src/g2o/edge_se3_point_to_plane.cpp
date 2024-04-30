#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <g2o/edge_se3_point_to_plane.hpp>

namespace g2o {

void EdgeSE3PointToPlane::computeError() {
  const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
  const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);

  Eigen::Matrix4d Ti = v1->estimate().matrix();
  Eigen::Vector4d Pj = v2->estimate().toVector();
  Eigen::Matrix4d Gij = _measurement;
  _error = Pj.transpose() * Ti * Gij * Ti.transpose() * Pj / 2;
}

bool EdgeSE3PointToPlane::read(std::istream& is) {
  Eigen::Matrix4d v;
  for (int i = 0; i < measurement().rows(); ++i)
    for (int j = 0; j < measurement().cols(); ++j) {
      is >> v(i, j);
    }
  setMeasurement(Eigen::Matrix4d(v));

  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgeSE3PointToPlane::write(std::ostream& os) const {
  Eigen::Matrix4d v = _measurement;

  for (int i = 0; i < measurement().rows(); ++i)
    for (int j = 0; j < measurement().cols(); ++j) {
      os << " " << v(i, j);
    }

  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}
}  // namespace g2o
