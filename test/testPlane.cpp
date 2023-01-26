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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/plane_analyzer.hpp>
#include <s_graphs/plane_mapper.hpp>
#include <s_graphs/plane_utils.hpp>

typedef pcl::PointXYZI PointT;

TEST(testPlane, ConvertPlaneToMap) {
  auto node = rclcpp::Node::make_shared("test_node");
  s_graphs::PlaneMapper plane_mapper(node);
  s_graphs::GraphSLAM graph_slam;
  Eigen::Isometry3d odom;
  odom.setIdentity();

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  s_graphs::KeyFrame::Ptr keyframe(
      new s_graphs::KeyFrame(rclcpp::Clock().now(), odom, 0.0, cloud));
  keyframe->node = graph_slam.add_se3_node(odom);
  Eigen::Vector4d local_plane;
  local_plane << 1, 0, 0, 10;
  g2o::Plane3D det_plane_body_frame(local_plane);
  g2o::Plane3D det_plane_map_frame =
      plane_mapper.convert_plane_to_map_frame(keyframe, local_plane);
  Eigen::Vector4d map_plane_vec = det_plane_map_frame.coeffs();
  EXPECT_EQ(map_plane_vec(0), 1);
  EXPECT_EQ(map_plane_vec(1), 0);
  EXPECT_EQ(map_plane_vec(2), 0);
  EXPECT_EQ(map_plane_vec(3) + 1, 10);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
