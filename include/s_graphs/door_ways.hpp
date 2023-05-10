#ifndef DOOR_WAYS_HPP
#define DOOR_WAYS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <g2o/vertex_doorway.hpp>

namespace g2o {
class VertexDoorWay;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id, Door's I.D for graph slam
 * @param prior_id, Door's I.D from prior knowledge
 * @param room1_id, I.D of 1st room
 * @param room2_id, I.D of 2nd room
 * @param door_pos_w, Door's position in WOrld frame
 * @param door_pos_r1, Door's position in 1st room's frame
 * @param door_pos_r1, Door's position in 2nd room's frame
 */
struct DoorWays {
 public:
  int id;
  int prior_id;
  Eigen::Vector3d door_pos_w, door_pos_r1, door_pose_r2;
  int room1_id, room2_id;
  g2o::VertexDoorWay* node;  // node instance
};

}  // namespace s_graphs
#endif  // DOOR_WAYS_HPP
