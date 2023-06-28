#ifndef STAIRS_HPP
#define STAIRS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>


namespace g2o {
//   class VertexStairStep;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id stair's I.D for graph slam
 * @param step_depth depth between risers
 * @param step_height height of a single step 
 * @param step_width  width of a single step
 * @param dist_x  distance in x-direction in Map frame
 * @param dist_y  distance in y-direction in Map frame
 */
struct DoorWays {
 public:
  int id;
  double step_depth;
  double step_height;
  double step_width;
  double dist_x, dist_y;
  // g2o::VertexStairStep* node;  // node instance
};

}  // namespace s_graphs
#endif  // STAIRS_HPP
