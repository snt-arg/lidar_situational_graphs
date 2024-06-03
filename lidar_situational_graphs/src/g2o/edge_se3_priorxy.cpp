#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <g2o/edge_se3_priorxy.hpp>

namespace g2o {
void EdgeSE3PriorXY::computeError() {
  const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

  Eigen::Vector2d estimate = v1->estimate().translation().head<2>();
  _error = estimate - _measurement;
}

bool EdgeSE3PriorXY::read(std::istream& is) {
  Eigen::Vector2d v;
  is >> v(0) >> v(1);
  setMeasurement(v);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}
bool EdgeSE3PriorXY::write(std::ostream& os) const {
  Eigen::Vector2d v = _measurement;
  os << v(0) << " " << v(1) << " ";
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}
}  // namespace g2o
