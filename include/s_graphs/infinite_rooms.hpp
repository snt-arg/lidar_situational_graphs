#ifndef INFINITE_ROOMS_HPP
#define INFINITE_ROOMS_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief Struct that holds information about an infinite room (infinite_room).
 *
 * @var id: Unique Id of the infinite room.
 * @var connected_id
 * @var connected_neighbour_ids
 * @var plane1, plane2: Planes that form the inifite room
 * @var plane1_id, plane2_id: Planes unique ids
 * @var neighbour_ids
 * @var cluster_center_node
 * @var node
 */
struct InfiniteRooms {
public:
  int id;
  int connected_id;
  std::vector<int> connected_neighbour_ids;
  g2o::Plane3D plane1, plane2;
  int plane1_id, plane2_id;
  std::vector<int> neighbour_ids;
  g2o::VertexRoomXYLB* cluster_center_node;
  g2o::VertexRoomXYLB* node;  // node instance
  bool sub_infinite_room;
};

}  // namespace s_graphs
#endif  // INFINITE_ROOMS_HPP
