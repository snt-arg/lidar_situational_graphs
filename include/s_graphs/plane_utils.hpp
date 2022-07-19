// SPDX-License-Identifier: BSD-2-Clause

#ifndef PLANE_UTILS_HPP
#define PLANE_UTILS_HPP

#include <Eigen/Dense>
#include <s_graphs/PlanesData.h>

namespace s_graphs {

enum plane_class : uint8_t {
  X_VERT_PLANE = 0,
  Y_VERT_PLANE = 1,
  HORT_PLANE = 2,
};

void correct_plane_d(int plane_type, s_graphs::PlaneData& plane) {
  if(plane_type == plane_class::X_VERT_PLANE) {
    plane.d = -1 * plane.d;
    double p_norm = plane.nx / fabs(plane.nx);
    plane.d = p_norm * plane.d;
  }

  if(plane_type == plane_class::Y_VERT_PLANE) {
    plane.d = -1 * plane.d;
    double p_norm = plane.ny / fabs(plane.ny);
    plane.d = p_norm * plane.d;
  }
  return;
}

void correct_plane_d(int plane_type, Eigen::Vector4d& plane) {
  if(plane_type == plane_class::X_VERT_PLANE) {
    plane(3) = -1 * plane(3);
    double p_norm = plane(0) / fabs(plane(0));
    plane(3) = p_norm * plane(3);
  }

  if(plane_type == plane_class::Y_VERT_PLANE) {
    plane(3) = -1 * plane(3);
    double p_norm = plane(1) / fabs(plane(1));
    plane(3) = p_norm * plane(3);
  }
  return;
}

}  // namespace s_graphs
#endif  // PLANE_UTILS_HPP
