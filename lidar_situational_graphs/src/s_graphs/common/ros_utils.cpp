#include "s_graphs/common/ros_utils.hpp"

#include "rclcpp/logger.hpp"

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
geometry_msgs::msg::TransformStamped matrix2transform(
    const rclcpp::Time& stamp,
    const Eigen::Matrix4f& pose,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::msg::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::msg::TransformStamped odom_trans;
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
geometry_msgs::msg::PoseStamped matrix2PoseStamped(const rclcpp::Time& stamp,
                                                   const Eigen::Matrix4f& pose,
                                                   const std::string& frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();

  geometry_msgs::msg::PoseStamped pose_stamped;
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
Eigen::Isometry3d pose2isometry(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  mat.linear() = Eigen::Quaterniond(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z)
                     .toRotationMatrix();
  return mat;
}

/**
 * @brief
 *
 * @param
 * @return
 */
Eigen::Isometry3d tf2isometry(const geometry_msgs::msg::TransformStamped& trans) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(trans.transform.translation.x,
                                      trans.transform.translation.y,
                                      trans.transform.translation.z);
  mat.linear() = Eigen::Quaterniond(trans.transform.rotation.w,
                                    trans.transform.rotation.x,
                                    trans.transform.rotation.y,
                                    trans.transform.rotation.z)
                     .toRotationMatrix();
  return mat;
}

/**
 * @brief
 *
 * @param
 * @return
 */
geometry_msgs::msg::Pose isometry2pose(const Eigen::Isometry3d& mat) {
  Eigen::Quaterniond quat(mat.linear());
  Eigen::Vector3d trans = mat.translation();

  geometry_msgs::msg::Pose pose;
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
Eigen::Isometry3d odom2isometry(const nav_msgs::msg::Odometry::SharedPtr& odom_msg) {
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

KeyFrame ROS2Keyframe(const situational_graphs_reasoning_msgs::msg::Keyframe& msg) {
  pcl::PointCloud<PointT>::Ptr cloud = pcl::PointCloud<PointT>::Ptr();
  // pcl::fromROSMsg(msg.pointcloud, *cloud);

  // TODO: add_accumulated_distance
  auto isometry = pose2isometry(msg.pose);
  KeyFrame keyframe(msg.header.stamp, isometry, 0.0, cloud);
  keyframe.node = new g2o::VertexSE3();
  keyframe.node->setId(msg.id);
  keyframe.node->setEstimate(isometry);
  return keyframe;
}

situational_graphs_reasoning_msgs::msg::Keyframe Keyframe2ROS(
    const KeyFrame& keyframe) {
  // TODO: add_accumulated_distance
  situational_graphs_reasoning_msgs::msg::Keyframe msg;
  msg.id = keyframe.id();
  msg.header.stamp = keyframe.stamp;
  msg.pose = isometry2pose(keyframe.estimate());
  // pcl::toROSMsg(*keyframe.cloud, msg.pointcloud);
  return msg;
}

Eigen::Matrix4f transformStamped2EigenMatrix(
    const geometry_msgs::msg::TransformStamped& transform_stamped) {
  // Extract the translation components
  double tx = transform_stamped.transform.translation.x;
  double ty = transform_stamped.transform.translation.y;
  double tz = transform_stamped.transform.translation.z;

  // Extract the rotation components (quaternion)
  double qx = transform_stamped.transform.rotation.x;
  double qy = transform_stamped.transform.rotation.y;
  double qz = transform_stamped.transform.rotation.z;
  double qw = transform_stamped.transform.rotation.w;

  // Create an Eigen 4x4 transformation matrix
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  // Set the translation components
  transform_matrix(0, 3) = tx;
  transform_matrix(1, 3) = ty;
  transform_matrix(2, 3) = tz;

  // Create an Eigen quaternion from the extracted quaternion components
  Eigen::Quaternionf quaternion(qw, qx, qy, qz);

  // Set the rotation components
  transform_matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

  return transform_matrix;
}

geometry_msgs::msg::TransformStamped eigenMatrixToTransformStamped(
    const Eigen::Matrix4f& matrix,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  geometry_msgs::msg::TransformStamped transform_stamped;

  // Extract translation
  Eigen::Vector3f translation = matrix.block<3, 1>(0, 3);

  // Extract rotation
  Eigen::Matrix3f rotation_matrix = matrix.block<3, 3>(0, 0);
  Eigen::Quaternionf quaternion(rotation_matrix);

  // Fill the TransformStamped message
  transform_stamped.header.stamp = rclcpp::Clock().now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();

  return transform_stamped;
}

}  // namespace s_graphs
