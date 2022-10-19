#ifndef FLOOR_HPP
#define FLOOR_HPP

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

struct Floors {
public:
  int id;
  int graph_id;
  int plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id;
  g2o::VertexRoomXYLB* node;  // node instance
};

}  // namespace s_graphs
#endif  // FLOORS_HPP
