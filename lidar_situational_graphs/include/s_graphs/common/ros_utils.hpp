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

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/common/keyframe.hpp"
#include "situational_graphs_reasoning_msgs/msg/keyframe.hpp"

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
    const std::string& child_frame_id);

/**
 * @brief
 *
 * @param
 * @return
 */
geometry_msgs::msg::PoseStamped matrix2PoseStamped(const rclcpp::Time& stamp,
                                                   const Eigen::Matrix4f& pose,
                                                   const std::string& frame_id);
/**
 * @brief
 *
 * @param
 * @return
 */
Eigen::Isometry3d pose2isometry(const geometry_msgs::msg::Pose& pose);

/**
 * @brief
 *
 * @param
 * @return
 */
Eigen::Isometry3d tf2isometry(const geometry_msgs::msg::TransformStamped& trans);

/**
 * @brief
 *
 * @param
 * @return
 */
geometry_msgs::msg::Pose isometry2pose(const Eigen::Isometry3d& mat);

/**
 * @brief
 *
 * @param
 * @return
 */
Eigen::Isometry3d odom2isometry(const nav_msgs::msg::Odometry::SharedPtr& odom_msg);

/**
 * @brief
 *
 * @param
 * @return
 */
KeyFrame ROS2Keyframe(const situational_graphs_reasoning_msgs::msg::Keyframe& msg);

/**
 * @brief
 *
 * @param
 * @return
 */
situational_graphs_reasoning_msgs::msg::Keyframe Keyframe2ROS(const KeyFrame& keyframe);

/**
 * @brief
 *
 */
Eigen::Matrix4f transformStamped2EigenMatrix(
    const geometry_msgs::msg::TransformStamped& transform_stamped);

/**
 * @brief
 *
 */
geometry_msgs::msg::TransformStamped eigenMatrixToTransformStamped(
    const Eigen::Matrix4f& matrix,
    const std::string& frame_id,
    const std::string& child_frame_id);

}  // namespace s_graphs
#endif  // ROS_UTILS_HPP
