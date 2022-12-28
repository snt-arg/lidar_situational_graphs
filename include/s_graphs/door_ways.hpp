#ifndef DOOR_WAYS_HPP
#define DOOR_WAYS_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace g2o {
class VertexDoorWayXYZ;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id, Door's I.D
 * @param room1_id, I.D of 1st room
 * @param room2_id, I.D of 2nd room
 * @param door_pos_w, Door's position in WOrld frame
 * @param door_pos_r1, Door's position in 1st room's frame
 * @param door_pos_r1, Door's position in 2nd room's frame
 */
struct DoorWays {
public:
  int id;
  Eigen::Vector3d door_pos_w, door_pos_r1, door_pose_r2;
  int room1_id, room2_id;
  g2o::VertexDoorWayXYZ* node;  // node instance
};

}  // namespace s_graphs
#endif  // DOOR_WAYS_HPP
