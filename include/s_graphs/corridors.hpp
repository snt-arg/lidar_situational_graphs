#ifndef CORRIDORS_HPP
#define CORRIDORS_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <g2o/vertex_corridor.hpp>
#include <g2o/vertex_room.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

struct Corridors {
public:
  int id;
  g2o::Plane3D plane1, plane2;
  int plane1_id, plane2_id;
  Eigen::Vector3d keyframe_trans;
  g2o::VertexRoomXYLB* node;  // node instance
};

}  // namespace s_graphs
#endif  // CORRIDORS_HPP
