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

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <memory>
#include <s_graphs/common/ros_utils.hpp>
#include <situational_graphs_msgs/msg/scan_matching_status.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/common/registrations.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace s_graphs {

class ScanMatchingOdometryNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNode() : Node("scan_matching_node") {
    initialize_params();
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    keyframe_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("enable_robot_odometry_init_guess", false);
    this->declare_parameter("enable_imu_frontend", false);
    if (this->get_parameter("enable_imu_frontend").get_parameter_value().get<bool>()) {
      msf_pose_sub =
          this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "msf_core/pose",
              1,
              std::bind(&ScanMatchingOdometryNode::msf_pose_callback,
                        this,
                        std::placeholders::_1));
      msf_pose_after_update_sub =
          this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "msf_core/pose_after_update",
              1,
              std::bind(&ScanMatchingOdometryNode::msf_pose_after_update_callback,
                        this,
                        std::placeholders::_1));
    }

    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_points",
        256,
        std::bind(
            &ScanMatchingOdometryNode::cloud_callback, this, std::placeholders::_1));
    read_until_pub = this->create_publisher<std_msgs::msg::Header>(
        "scan_matching_odometry/read_until", 32);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 32);
    trans_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "scan_matching_odometry/transform", 32);
    status_pub =
        this->create_publisher<situational_graphs_msgs::msg::ScanMatchingStatus>(
            "scan_matching_odometry/status", 8);
    aligned_points_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_points", 32);
  }

 private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    this->declare_parameter("points_topic", "velodyne_points");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("robot_odom_frame_id", "robot_odom");
    this->declare_parameter("publish_tf", true);

    points_topic =
        this->get_parameter("points_topic").get_parameter_value().get<std::string>();
    odom_frame_id =
        this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>();
    robot_odom_frame_id = this->get_parameter("robot_odom_frame_id")
                              .get_parameter_value()
                              .get<std::string>();

    std::string ns = this->get_namespace();
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      odom_frame_id = ns_prefix + "/" + odom_frame_id;
      robot_odom_frame_id = ns_prefix + "/" + odom_frame_id;
    }

    publish_tf = this->get_parameter("publish_tf").get_parameter_value().get<bool>();

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    this->declare_parameter("keyframe_delta_trans", 0.25);
    this->declare_parameter("keyframe_delta_angle", 0.15);
    this->declare_parameter("keyframe_delta_time", 1.0);
    keyframe_delta_trans =
        this->get_parameter("keyframe_delta_trans").get_parameter_value().get<double>();
    keyframe_delta_angle =
        this->get_parameter("keyframe_delta_angle").get_parameter_value().get<double>();
    keyframe_delta_time =
        this->get_parameter("keyframe_delta_time").get_parameter_value().get<double>();

    // Registration validation by thresholding
    this->declare_parameter("transform_thresholding", false);
    this->declare_parameter("max_acceptable_trans", 1.0);
    this->declare_parameter("max_acceptable_angle", 1.0);
    transform_thresholding =
        this->get_parameter("transform_thresholding").get_parameter_value().get<bool>();
    max_acceptable_trans =
        this->get_parameter("max_acceptable_trans").get_parameter_value().get<double>();
    max_acceptable_angle =
        this->get_parameter("max_acceptable_angle").get_parameter_value().get<double>();

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    this->declare_parameter("downsample_method", "VOXELGRID");
    this->declare_parameter("downsample_resolution", 0.1);

    std::string downsample_method = this->get_parameter("downsample_method")
                                        .get_parameter_value()
                                        .get<std::string>();
    double downsample_resolution = this->get_parameter("downsample_resolution")
                                       .get_parameter_value()
                                       .get<double>();
    if (downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      pcl::VoxelGrid<PointT>::Ptr voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(
          downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if (downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution
                << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(
          new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(
          downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if (downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")"
                  << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    this->declare_parameter("registration_method", "NDT_OMP");
    this->declare_parameter("reg_num_threads", 0);
    this->declare_parameter("reg_transformation_epsilon", 0.01);
    this->declare_parameter("reg_maximum_iterations", 64);
    this->declare_parameter("reg_max_correspondence_distance", 2.5);
    this->declare_parameter("reg_correspondence_randomness", 20);
    this->declare_parameter("reg_resolution", 1.0);
    this->declare_parameter("reg_use_reciprocal_correspondences", false);
    this->declare_parameter("reg_max_optimizer_iterations", 20);
    this->declare_parameter("reg_nn_search_method", "DIRECT7");
    s_graphs::registration_params params;
    params = {
        this->get_parameter("registration_method")
            .get_parameter_value()
            .get<std::string>(),
        this->get_parameter("reg_num_threads").get_parameter_value().get<int>(),
        this->get_parameter("reg_transformation_epsilon")
            .get_parameter_value()
            .get<double>(),
        this->get_parameter("reg_maximum_iterations").get_parameter_value().get<int>(),
        this->get_parameter("reg_max_correspondence_distance")
            .get_parameter_value()
            .get<double>(),
        this->get_parameter("reg_correspondence_randomness")
            .get_parameter_value()
            .get<int>(),
        this->get_parameter("reg_resolution").get_parameter_value().get<double>(),
        this->get_parameter("reg_use_reciprocal_correspondences")
            .get_parameter_value()
            .get<bool>(),
        this->get_parameter("reg_max_optimizer_iterations")
            .get_parameter_value()
            .get<int>(),
        this->get_parameter("reg_nn_search_method")
            .get_parameter_value()
            .get<std::string>()};

    registration = select_registration_method(params);
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    if (!rclcpp::ok()) {
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::msg::Header::SharedPtr read_until(new std_msgs::msg::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = rclcpp::Time(cloud_msg->header.stamp) + rclcpp::Duration(1, 0);
    read_until_pub->publish(*read_until);

    read_until->frame_id = "filtered_points";
    read_until_pub->publish(*read_until);
  }

  void msf_pose_callback(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    msf_pose = pose_msg;
  }

  void msf_pose_after_update_callback(
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
    msf_pose_after_update = pose_msg;
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(
      const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4f matching(const rclcpp::Time& stamp,
                           const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if (!keyframe) {
      prev_time = rclcpp::Time();
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    registration->setInputSource(filtered);

    std::string msf_source;
    Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

    if (this->get_parameter("enable_imu_frontend").get_parameter_value().get<bool>()) {
      if (msf_pose && rclcpp::Time(msf_pose->header.stamp) > keyframe_stamp &&
          msf_pose_after_update &&
          rclcpp::Time(msf_pose_after_update->header.stamp) > keyframe_stamp) {
        Eigen::Isometry3d pose0 = pose2isometry(msf_pose_after_update->pose.pose);
        Eigen::Isometry3d pose1 = pose2isometry(msf_pose->pose.pose);
        Eigen::Isometry3d delta = pose0.inverse() * pose1;

        msf_source = "imu";
        msf_delta = delta.cast<float>();
      } else {
        std::cerr << "msf data is too old" << std::endl;
      }
    } else if (this->get_parameter("enable_robot_odometry_init_guess")
                   .get_parameter_value()
                   .get<bool>()) {
      geometry_msgs::msg::TransformStamped transform_msg;
      if (tf_buffer->canTransform(cloud->header.frame_id,
                                  stamp,
                                  cloud->header.frame_id,
                                  prev_time,
                                  robot_odom_frame_id)) {
        try {
          transform_msg = tf_buffer->lookupTransform(cloud->header.frame_id,
                                                     stamp,
                                                     cloud->header.frame_id,
                                                     prev_time,
                                                     robot_odom_frame_id);
          msf_source = "odometry";
          msf_delta = tf2isometry(transform_msg).cast<float>();
        } catch (const tf2::TransformException& ex) {
          RCLCPP_INFO(this->get_logger(),
                      "Could not transform %s to %s: %s",
                      cloud->header.frame_id.c_str(),
                      robot_odom_frame_id.c_str(),
                      ex.what());
        }
      } else if (tf_buffer->canTransform(
                     cloud->header.frame_id,
                     rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
                     cloud->header.frame_id,
                     prev_time,
                     robot_odom_frame_id)) {
        transform_msg = tf_buffer->lookupTransform(
            cloud->header.frame_id,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
            cloud->header.frame_id,
            prev_time,
            robot_odom_frame_id);
        msf_source = "odometry";
        msf_delta = tf2isometry(transform_msg).cast<float>();
      }
    }

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, prev_trans * msf_delta.matrix());

    publish_scan_matching_status(
        stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    if (!registration->hasConverged()) {
      RCLCPP_INFO(this->get_logger(), "scan matching has not converged!!");
      RCLCPP_INFO(this->get_logger(),
                  "ignore this frame: %s ",
                  std::to_string(stamp.seconds()).c_str());
      return keyframe_pose * prev_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;

    if (transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

      if (dx > max_acceptable_trans || da > max_acceptable_angle) {
        RCLCPP_INFO(
            this->get_logger(), "too large transform!!  %f [m], %f [rad]", dx, da);
        RCLCPP_INFO(this->get_logger(),
                    "ignore this frame %s ",
                    std::to_string(stamp.seconds()).c_str());
        return keyframe_pose * prev_trans;
      }
    }

    prev_time = stamp;
    prev_trans = trans;

    auto keyframe_trans =
        matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster->sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).seconds();
    if (delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle ||
        delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_time = stamp;
      prev_trans.setIdentity();
    }

    if (aligned_points_pub->get_subscription_count() > 0) {
      pcl::transformPointCloud(*cloud, *aligned, odom);
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header.frame_id = odom_frame_id;
      aligned_points_pub->publish(aligned_msg);
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const rclcpp::Time& stamp,
                        const std::string& base_frame_id,
                        const Eigen::Matrix4f& pose) {
    // publish transform stamped for IMU integration
    geometry_msgs::msg::TransformStamped odom_trans =
        matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    trans_pub->publish(odom_trans);

    // broadcast the transform over tf
    if (publish_tf) odom_broadcaster->sendTransform(odom_trans);

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub->publish(odom);
  }

  /**
   * @brief publish scan matching status
   */
  void publish_scan_matching_status(const rclcpp::Time& stamp,
                                    const std::string& frame_id,
                                    pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned,
                                    const std::string& msf_source,
                                    const Eigen::Isometry3f& msf_delta) {
    if (status_pub->get_subscription_count() == 0) {
      return;
    }
    situational_graphs_msgs::msg::ScanMatchingStatus status;
    status.header.frame_id = frame_id;
    status.header.stamp = stamp;
    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (size_t i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(
          pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    status.relative_pose = isometry2pose(
        Eigen::Isometry3f(registration->getFinalTransformation()).cast<double>());

    if (!msf_source.empty()) {
      status.prediction_labels.resize(1);
      status.prediction_labels[0].data = msf_source;

      status.prediction_errors.resize(1);
      Eigen::Isometry3f error =
          Eigen::Isometry3f(registration->getFinalTransformation()).inverse() *
          msf_delta;
      status.prediction_errors[0] = isometry2pose(error.cast<double>());
    }

    status_pub->publish(status);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      msf_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      msf_pose_after_update_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr trans_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::ScanMatchingStatus>::SharedPtr
      status_pub;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> keyframe_broadcaster;

  std::string points_topic;
  std::string odom_frame_id;
  std::string robot_odom_frame_id;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msf_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msf_pose_after_update;
  bool publish_tf;

  rclcpp::Time prev_time;
  Eigen::Matrix4f prev_trans;     // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;  // keyframe pose
  rclcpp::Time keyframe_stamp;    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::ScanMatchingOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
