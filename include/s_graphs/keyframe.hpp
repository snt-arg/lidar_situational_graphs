// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include "rclcpp/rclcpp.hpp"
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

  /**
   * @brief Constructor for class KeyFrame
   *
   * @param stamp
   * @param odom
   * @param accum_distance
   * @param cloud
   * @return
   */
  KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  /**
   * @brief Constructor for class KeyFrame
   *
   *
   * @param directory
   * @param graph
   */
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  /**
   * @brief Saves the keyframe into a given directory.
   *
   * @param directory
   */
  void save(const std::string& directory);

  /**
   * @brief Loads the keyframes from a directory into the graph pointer.
   *
   * @param directory
   * @param graph
   * @return Success or failure
   */
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  /**
   * @brief
   *
   * @return
   */
  long id() const;

  /**
   * @brief
   *
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

  /**
   * @brief Constructor for class KeyFramesnapshot
   *
   * @param key
   */
  KeyFrameSnapshot(const KeyFrame::Ptr& key);

  /**
   * @brief Constructor for class KeyFramesnapshot
   *
   * @param key
   */
  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

}  // namespace s_graphs

#endif  // KEYFRAME_HPP
