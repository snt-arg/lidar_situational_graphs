/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/common/planes.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"

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
   * @param floor_level
   * @return
   */
  KeyFrame(const rclcpp::Time& stamp,
           const Eigen::Isometry3d& odom,
           double accum_distance,
           const pcl::PointCloud<PointT>::Ptr& cloud,
           const int floor_level = 0,
           const int session_id = 0);

  /**
   * @brief Constructor for class KeyFrame
   *
   *
   * @param directory
   * @param graph
   */
  KeyFrame(const std::string& directory,
           const std::shared_ptr<GraphSLAM>& covisibility_graph);

  KeyFrame(const KeyFrame::Ptr& key);

  virtual ~KeyFrame();

  /**
   * @brief Construct a new KeyFrame::set_dense_cloud object
   *
   * @param cloud
   */
  void set_dense_cloud(const pcl::PointCloud<PointT>::Ptr& cloud);

  /**
   * @brief Saves the keyframe into a given directory.
   *
   * @param directory
   * @param directory
   */
  void save(const std::string& directory, const int& sequential_id);

  /**
   * @brief Loads the keyframes from a directory into the graph pointer.
   *
   * @param directory
   * @param graph
   * @return Success or failure
   */
  bool load(const std::string& directory,
            const std::shared_ptr<GraphSLAM>& covisibility_graph);

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
  rclcpp::Time stamp;                  // timestamp
  Eigen::Isometry3d odom;              // odometry (estimated by scan_matching_odometry)
  double accum_distance;               // accumulated distance from the first node (by
                                       // scan_matching_odometry)
  pcl::PointCloud<PointT>::Ptr cloud;  // point cloud
  pcl::PointCloud<PointT>::Ptr dense_cloud;  // denser point cloud

  std::vector<int> x_plane_ids, y_plane_ids,
      hort_plane_ids;  // list of planes associated with the keyframe

  int floor_level;  // floor level that the keyframe belongs
  int session_id;   // current session (multi-session) the keframe belongs to

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
  KeyFrameSnapshot(const Eigen::Isometry3d& pose,
                   const pcl::PointCloud<PointT>::ConstPtr& cloud,
                   const bool marginalized = false,
                   const int floor_level = 0);

  ~KeyFrameSnapshot();

 public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
  bool k_marginalized = false;              // whether keyframe is marginalized
  int floor_level = 0;
};

}  // namespace s_graphs

#endif  // KEYFRAME_HPP
