#ifndef CORRIDORS_HPP
#define CORRIDORS_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

struct Corridors {
public:
  int id;
  g2o::Plane3D plane1, plane2;
  int plane1_id, plane2_id;
  Eigen::Vector3d keyframe_trans;
  g2o::VertexSE3* node;                       // node instance
};

}
#endif  // CORRIDORS_HPP
