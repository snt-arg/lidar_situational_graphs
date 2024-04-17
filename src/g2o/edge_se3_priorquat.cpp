#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <g2o/edge_se3_priorquat.hpp>

namespace g2o {

void EdgeSE3PriorQuat::computeError() {
  const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

  Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());

  if (_measurement.coeffs().dot(estimate.coeffs()) < 0.0) {
    estimate.coeffs() = -estimate.coeffs();
  }
  _error = estimate.vec() - _measurement.vec();
}

void EdgeSE3PriorQuat::setMeasurement(const Eigen::Quaterniond& m) {
  _measurement = m;
  if (m.w() < 0.0) {
    _measurement.coeffs() = -m.coeffs();
  }
}

bool EdgeSE3PriorQuat::read(std::istream& is) {
  Eigen::Quaterniond q;
  is >> q.w() >> q.x() >> q.y() >> q.z();
  setMeasurement(q);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgeSE3PriorQuat::write(std::ostream& os) const {
  Eigen::Quaterniond q = _measurement;
  os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}
}  // namespace g2o
