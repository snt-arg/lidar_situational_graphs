// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include <s_graphs/planes.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using PointNormal = pcl::PointXYZRGBNormal;

  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  /**
   * @brief
   *
   * @param
   * @return
   */
  void save(const std::string& directory);

  /**
   * @brief
   *
   * @param
   * @return
   */
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  /**
   * @brief
   *
   * @param
   * @return
   */
  long id() const;

  /**
   * @brief
   *
   * @param
   * @return
   */
  Eigen::Isometry3d estimate() const;

public:
  ros::Time stamp;                                            // timestamp
  Eigen::Isometry3d odom;                                     // odometry (estimated by scan_matching_odometry)
  double accum_distance;                                      // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;                    // point cloud
  pcl::PointCloud<PointNormal>::Ptr cloud_seg_body;           // semantically segmented pointcloud
  std::vector<int> x_plane_ids, y_plane_ids, hort_plane_ids;  // list of planes associated with the keyframe

  boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS

  boost::optional<Eigen::Vector3d> acceleration;    //
  boost::optional<Eigen::Quaterniond> orientation;  //

  g2o::VertexSE3* node;  // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}  // namespace s_graphs

#endif  // KEYFRAME_HPP
