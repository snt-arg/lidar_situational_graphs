#ifndef ROOMS_HPP
#define ROOMS_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace g2o {
class VertexRoomXYLB;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

struct Rooms {
public:
  int id;
  g2o::Plane3D plane_x1, plane_x2, plane_y1, plane_y2;
  int plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id;
  bool x_axis_identified, y_axis_identified;
  bool room_identified;
  g2o::VertexRoomXYLB* node;                       // node instance
};

}
#endif  // ROOMS_HPP