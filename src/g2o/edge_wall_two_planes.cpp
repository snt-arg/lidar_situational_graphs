#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_wall_two_planes.hpp>

#include "g2o/vertex_wall.hpp"
namespace g2o {

/*   Define Wall edge with wall surfaces here*/

void EdgeWall2Planes::computeError() {
  const VertexWallXYZ* v1 = static_cast<const VertexWallXYZ*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  const VertexPlane* v3 = static_cast<const VertexPlane*>(_vertices[2]);

  Eigen::Vector3d wall_center = v1->estimate();
  Eigen::Vector4d plane1 = v2->estimate().coeffs();
  Eigen::Vector4d plane2 = v3->estimate().coeffs();

  correct_plane_direction(plane1);
  correct_plane_direction(plane2);

  Eigen::Vector3d estimated_wall_center;
  if (fabs(plane1(3)) > fabs(plane2(3))) {
    estimated_wall_center =
        (0.5 * (fabs(plane1(3)) * plane1.head(3) - fabs(plane2(3)) * plane2.head(3))) +
        fabs(plane2(3)) * plane2.head(3);
  } else {
    estimated_wall_center =
        (0.5 * (fabs(plane2(3)) * plane2.head(3) - fabs(plane1(3)) * plane1.head(3))) +
        fabs(plane1(3)) * plane1.head(3);
  }

  Eigen::Vector3d estimated_wall_center_normalized =
      estimated_wall_center.head(2) / estimated_wall_center.norm();
  Eigen::Vector3d final_wall_center =
      estimated_wall_center.head(2) +
      (_wall_point - (_wall_point.dot(estimated_wall_center_normalized)) *
                         estimated_wall_center_normalized);

  _error = wall_center - final_wall_center;
}

bool EdgeWall2Planes::read(std::istream& is) {
  Eigen::Vector3d v;
  is >> v(0) >> v(1) >> v(2);

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

bool EdgeWall2Planes::write(std::ostream& os) const {
  Eigen::Vector3d v = _measurement;
  os << v(0) << " " << v(1) << " " << v(2) << " ";

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgeWall2Planes::correct_plane_direction(Eigen::Vector4d& plane) {
  if (plane(3) > 0) {
    plane(0) = -1 * plane(0);
    plane(1) = -1 * plane(1);
    plane(2) = -1 * plane(2);
    plane(3) = -1 * plane(3);
  } else {
    plane = plane;
  }
}

}  // namespace g2o
