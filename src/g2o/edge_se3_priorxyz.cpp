#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <g2o/edge_se3_priorxyz.hpp>

namespace g2o {

void EdgeSE3PriorXYZ::computeError() {
  const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

  Eigen::Vector3d estimate = v1->estimate().translation();
  _error = estimate - _measurement;
}

bool EdgeSE3PriorXYZ::read(std::istream& is) {
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
bool EdgeSE3PriorXYZ::write(std::ostream& os) const {
  Eigen::Vector3d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " ";
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}
}  // namespace g2o
