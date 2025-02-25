#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <g2o/edge_se3_two_planes.hpp>
#include <g2o/vertex_deviation.hpp>

namespace g2o {

void EdgeSE3PlanePlane::computeError() {
  const g2o::VertexDeviation* v1 =
      static_cast<const g2o::VertexDeviation*>(_vertices[0]);
  const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);
  const g2o::VertexPlane* v3 = static_cast<const g2o::VertexPlane*>(_vertices[2]);

  g2o::Plane3D a_graph_plane = v2->estimate();
  g2o::Plane3D s_graph_plane = v3->estimate();

  Eigen::Isometry3d deviation = v1->estimate();
  Eigen::Isometry3d dev_inverse = v1->estimate().inverse();
  g2o::Plane3D modified_s_graph_plane = dev_inverse * v3->estimate();
  _error = modified_s_graph_plane.ominus(v2->estimate());
  // if (v2->id() == 18 || v2->id() == 12) {
  //   std::cout << "a_graph plane fixed : " << v2->fixed() << std::endl;
  //   std::cout << "s_graph plane fixed : " << v3->fixed() << std::endl;
  //   std::cout << "error : " << std::endl;
  //   std::cout << _error << std::endl;
  //   std::cout << "deviation : " << std::endl;
  //   std::cout << v1->estimate().matrix() << std::endl;
  //   std::cout << "a_graph_plane : " << std::endl;
  //   std::cout << v2->estimate().toVector() << std::endl;
  //   std::cout << "s_graph_plane : " << std::endl;
  //   std::cout << v3->estimate().toVector() << std::endl;
  // }
}
bool EdgeSE3PlanePlane::read(std::istream& is) {
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
bool EdgeSE3PlanePlane::write(std::ostream& os) const {
  Eigen::Vector3d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " ";
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
  return os.good();
}

}  // namespace g2o
