#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

#include <Eigen/Dense>
#include <g2o/edge_plane.hpp>

namespace g2o {

void EdgePlaneParallel::computeError() {
  const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);

  double num = v1->estimate().normal().dot(v2->estimate().normal());
  double den = v1->estimate().normal().norm() * (v2->estimate().normal().norm());

  _error[0] = acos(fabs(num) / den);
}
bool EdgePlaneParallel::read(std::istream& is) {
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) {
    is >> v[i];
  }

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

bool EdgePlaneParallel::write(std::ostream& os) const {
  for (int i = 0; i < 3; ++i) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void EdgePlanePerpendicular::computeError() {
  const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);

  Eigen::Vector3d normal1 = v1->estimate().normal().normalized();
  Eigen::Vector3d normal2 = v2->estimate().normal().normalized();

  _error[0] = normal1.dot(normal2);
}

bool EdgePlanePerpendicular::read(std::istream& is) {
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) {
    is >> v[i];
  }

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

bool EdgePlanePerpendicular::write(std::ostream& os) const {
  for (int i = 0; i < 3; ++i) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

void Edge2Planes::computeError() {
  const VertexPlane* v1 = static_cast<const VertexPlane*>(_vertices[0]);
  const VertexPlane* v2 = static_cast<const VertexPlane*>(_vertices[1]);
  g2o::Plane3D plane1 = v1->estimate();
  g2o::Plane3D plane2 = v2->estimate();

  _error = plane1.ominus(plane2);
}

bool Edge2Planes::read(std::istream& is) {
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) {
    is >> v[i];
  }

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

bool Edge2Planes::write(std::ostream& os) const {
  for (int i = 0; i < 3; ++i) {
    os << _measurement[i] << " ";
  }

  for (int i = 0; i < information().rows(); ++i) {
    for (int j = i; j < information().cols(); ++j) {
      os << " " << information()(i, j);
    };
  }
  return os.good();
}

}  // namespace g2o
