// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace s_graphs {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 *
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static geometry_msgs::PoseStamped matrix2PoseStamped(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose.position.x = pose(0, 3);
  pose_stamped.pose.position.y = pose(1, 3);
  pose_stamped.pose.position.z = pose(2, 3);
  pose_stamped.pose.orientation.w = quat.w();
  pose_stamped.pose.orientation.x = quat.x();
  pose_stamped.pose.orientation.y = quat.y();
  pose_stamped.pose.orientation.z = quat.z();

  return pose_stamped;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static Eigen::Isometry3d pose2isometry(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  mat.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return mat;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static Eigen::Isometry3d tf2isometry(const tf::StampedTransform& trans) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
  mat.linear() = Eigen::Quaterniond(trans.getRotation().w(), trans.getRotation().x(), trans.getRotation().y(), trans.getRotation().z()).toRotationMatrix();
  return mat;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static geometry_msgs::Pose isometry2pose(const Eigen::Isometry3d& mat) {
  Eigen::Quaterniond quat(mat.linear());
  Eigen::Vector3d trans = mat.translation();

  geometry_msgs::Pose pose;
  pose.position.x = trans.x();
  pose.position.y = trans.y();
  pose.position.z = trans.z();
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  return pose;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static geometry_msgs::PoseStamped isometry2pose_stamped(const Eigen::Isometry3d& mat) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = mat.translation().x();
  pose_stamped.pose.position.y = mat.translation().y();
  pose_stamped.pose.position.z = mat.translation().z();
  Eigen::Quaterniond quat(mat.linear());
  tf2::Quaternion tf_quat;
  tf_quat.setX(quat.x());
  tf_quat.setY(quat.y());
  tf_quat.setZ(quat.z());
  tf_quat.setW(quat.w());
  pose_stamped.pose.orientation.x = tf_quat.getX();
  pose_stamped.pose.orientation.y = tf_quat.getY();
  pose_stamped.pose.orientation.z = tf_quat.getZ();
  pose_stamped.pose.orientation.w = tf_quat.getW();
  return pose_stamped;
}

/**
 * @brief
 *
 * @param
 * @return
 */
static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

}  // namespace s_graphs

#endif  // ROS_UTILS_HPP
