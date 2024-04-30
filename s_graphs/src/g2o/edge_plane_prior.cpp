#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_plane_prior.hpp>

namespace g2o {
void EdgePlanePriorNormal::computeError() {
  const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
  Eigen::Vector3d normal = v1->estimate().normal();

  if (normal.dot(_measurement) < 0.0) {
    normal = -normal;
  }

  _error = normal - _measurement;
}

bool EdgePlanePriorNormal::read(std::istream& is) {
  Eigen::Vector3d v;
  is >> v(0) >> v(1) >> v(2);
  setMeasurement(v);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgePlanePriorNormal::write(std::ostream& os) const {
  Eigen::Vector3d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " ";
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}

void EdgePlanePriorDistance::computeError() {
  const g2o::VertexPlane* v1 = static_cast<const g2o::VertexPlane*>(_vertices[0]);
  _error[0] = _measurement - v1->estimate().distance();
}

bool EdgePlanePriorDistance::read(std::istream& is) {
  is >> _measurement;
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgePlanePriorDistance::write(std::ostream& os) const {
  os << _measurement;
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}
}  // namespace g2o
