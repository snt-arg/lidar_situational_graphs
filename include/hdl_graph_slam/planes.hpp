#ifndef PLANES_HPP
#define PLANES_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

struct VerticalPlanes {
public:
  using PointNormal = pcl::PointXYZRGBNormal;

public:
  int id;
  Eigen::Vector4d coefficients;
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_body;  // segmented points of the plane in local body frame
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_map;  // segmented points of the plane in global map frame
  g2o::VertexPlane* node;                       // node instance
};

struct HorizontalPlanes {
public:
  using PointNormal = pcl::PointXYZRGBNormal;

public:
  int id;
  Eigen::Vector4d coefficients;
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_body;  // segmented points of the plane in local body frame
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_map;  // segmented points of the plane in global map frame
  g2o::VertexPlane* node;                       // node instance
};

}  // namespace hdl_graph_slam

#endif  // PLANES_HPP
