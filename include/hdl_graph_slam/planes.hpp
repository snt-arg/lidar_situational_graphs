#ifndef PLANES_HPP
#define PLANES_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/vertex_plane.h>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o


namespace hdl_graph_slam {

struct VerticalPlanes {
public:
    int id;
    Eigen::Vector4d coefficients;
    g2o::VertexPlane* node;  // node instance

};

struct HorizontalPlanes {
public:
    int id;
    Eigen::Vector4d coefficients;
    g2o::VertexPlane* node; // node instance
};

}  // namespace hdl_graph_slam

#endif  // PLANES_HPP
