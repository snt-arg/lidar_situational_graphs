#ifndef ROOMS_HPP
#define ROOMS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

namespace g2o {
class VertexRoomXYLB;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id
 * @param connected_id
 * @param connected_neighbour_ids
 * @param plane_x1, plane_x2, plane_y1, plane_y2
 * @param plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @param neighbour_ids
 * @param node
 * @param sub_room
 */
struct Rooms {
 public:
  int id;
  int connected_id;
  std::vector<int> connected_neighbour_ids;
  g2o::Plane3D plane_x1, plane_x2, plane_y1, plane_y2;
  int plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id;
  std::vector<int> neighbour_ids;
  g2o::VertexRoomXYLB* node;  // node instance
  bool sub_room;
};

}  // namespace s_graphs
#endif  // ROOMS_HPP
