#ifndef PLANES_HPP
#define PLANES_HPP

#include <Eigen/Eigen>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

struct VerticalPlanes {
public:
  using PointNormal = pcl::PointXYZRGBNormal;

public:
  int id;
  g2o::Plane3D plane;
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_body;                   // segmented points of the plane in local body frame
  std::vector<pcl::PointCloud<PointNormal>::Ptr> cloud_seg_body_vec;  // vector of segmented points of the plane in local body frame
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_map;                    // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;                                         // covariance of the landmark
  bool parallel_pair;                                                 // checking if the plane has parallel pair
  g2o::VertexSE3* keyframe_node;                                      // keyframe node instance
  std::vector<g2o::VertexSE3*> keyframe_node_vec;                     // vector keyframe node instance
  g2o::VertexPlane* plane_node;                                       // node instance
  std::vector<double> color;
};

struct HorizontalPlanes {
public:
  using PointNormal = pcl::PointXYZRGBNormal;

public:
  int id;
  g2o::Plane3D plane;
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_body;                   // segmented points of the plane in local body frame
  std::vector<pcl::PointCloud<PointNormal>::Ptr> cloud_seg_body_vec;  // vector of segmented points of the plane in local body frame
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_map;                    // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;                                         // covariance of the landmark
  bool parallel_pair;                                                 // checking if the plane has parallel pair
  g2o::VertexSE3* keyframe_node;                                      // keyframe node instance
  std::vector<g2o::VertexSE3*> keyframe_node_vec;                     // vector keyframe node instance
  g2o::VertexPlane* plane_node;                                       // node instance
  std::vector<double> color;
};

}  // namespace s_graphs

#endif  // PLANES_HPP
