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
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/plane_mapper.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>

typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGBNormal PointNormal;

class TestPlane : public ::testing::Test {
 public:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<s_graphs::PlaneMapper> plane_mapper;
  std::shared_ptr<s_graphs::GraphSLAM> graph_slam;
  Eigen::Isometry3d odom;
  s_graphs::KeyFrame::Ptr keyframe;
  g2o::Plane3D det_plane_map_frame;
  Eigen::Vector4d map_plane_vec;
  pcl::PointCloud<PointT>::Ptr cloud;
  std::unordered_map<int, s_graphs::VerticalPlanes> x_vert_planes;
  std::unordered_map<int, s_graphs::VerticalPlanes> y_vert_planes;
  std::unordered_map<int, s_graphs::HorizontalPlanes> hort_planes;

  void SetUp() override {
    node = rclcpp::Node::make_shared("test_node");
    node->declare_parameter("use_point_to_plane", false);
    node->declare_parameter("plane_information", 0.01);
    node->declare_parameter("plane_dist_threshold", 0.35);
    node->declare_parameter("plane_points_dist", 0.1);
    node->declare_parameter("min_plane_points", 100);

    graph_slam = std::make_shared<s_graphs::GraphSLAM>();
    plane_mapper = std::make_shared<s_graphs::PlaneMapper>(node);
    cloud = boost::make_shared<pcl::PointCloud<PointT>>();
  }

  void testConvertPlaneToMap() {
    odom.setIdentity();
    keyframe =
        std::make_shared<s_graphs::KeyFrame>(rclcpp::Clock().now(), odom, 0.0, cloud);
    keyframe->node = graph_slam->add_se3_node(odom);
    Eigen::Vector4d local_plane;
    local_plane << 1, 0, 0, 10;
    g2o::Plane3D det_plane_body_frame(local_plane);
    det_plane_map_frame =
        plane_mapper->convert_plane_to_map_frame(keyframe, local_plane);
    map_plane_vec = det_plane_map_frame.coeffs();
  }

  void testConvertPlanePointsToMap() {
    s_graphs::VerticalPlanes x_vert_plane;
    pcl::PointCloud<PointNormal>::Ptr cloud_seg_body(
        new pcl::PointCloud<PointNormal>());
    PointNormal point;
    point.x = 1;
    point.y = 2;
    point.z = 3;
    cloud_seg_body->points.push_back(point);
    x_vert_plane.cloud_seg_body_vec.push_back(cloud_seg_body);

    odom.setIdentity();
    s_graphs::KeyFrame::Ptr keyframe(
        new s_graphs::KeyFrame(rclcpp::Clock().now(), odom, 0.0, cloud));

    keyframe->node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
    x_vert_plane.keyframe_node_vec.push_back(keyframe->node);
    x_vert_planes.insert({x_vert_plane.id, x_vert_plane});
    plane_mapper->convert_plane_points_to_map(
        x_vert_planes, y_vert_planes, hort_planes);
  }

  int testAssociatePlanes() {
    g2o::Plane3D det_plane;
    Eigen::Vector4d det_plane_coeffs;
    det_plane_coeffs << 1, 0, 0, 9.9;
    det_plane = det_plane_coeffs;

    odom.setIdentity();
    s_graphs::KeyFrame::Ptr keyframe(
        new s_graphs::KeyFrame(rclcpp::Clock().now(), odom, 0.0, cloud));
    keyframe->node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());

    s_graphs::VerticalPlanes x_vert_plane;
    x_vert_plane.id = 1;
    Eigen::Vector4d local_plane;
    local_plane << 1, 0, 0, 10;
    g2o::Plane3D mapped_plane(local_plane);
    x_vert_plane.plane = mapped_plane;
    x_vert_plane.keyframe_node = keyframe->node;
    x_vert_plane.cloud_seg_body = boost::make_shared<pcl::PointCloud<PointNormal>>();
    x_vert_plane.cloud_seg_map = boost::make_shared<pcl::PointCloud<PointNormal>>();
    x_vert_plane.keyframe_node_vec.push_back(keyframe->node);
    x_vert_planes.insert({x_vert_plane.id, x_vert_plane});

    int matched_plane =
        plane_mapper->associate_plane(s_graphs::PlaneUtils::plane_class::X_VERT_PLANE,
                                      keyframe,
                                      det_plane,
                                      keyframe->cloud_seg_body,
                                      x_vert_planes,
                                      y_vert_planes,
                                      hort_planes);
    return matched_plane;
  }
};

TEST_F(TestPlane, ConvertPlaneToMap) {
  this->testConvertPlaneToMap();
  EXPECT_EQ(map_plane_vec(0), 1);
  EXPECT_EQ(map_plane_vec(1), 0);
  EXPECT_EQ(map_plane_vec(2), 0);
  EXPECT_EQ(map_plane_vec(3), 10);
}

TEST_F(TestPlane, ConvertPlanePointsToMap) {
  this->testConvertPlanePointsToMap();

  ASSERT_EQ(x_vert_planes.size(), 1);
  ASSERT_EQ(y_vert_planes.size(), 0);
  ASSERT_EQ(hort_planes.size(), 0);

  ASSERT_EQ(x_vert_planes[0].cloud_seg_map->points.size(), 1);
  EXPECT_EQ(x_vert_planes[0].cloud_seg_map->points[0].x, 1);
  EXPECT_EQ(x_vert_planes[0].cloud_seg_map->points[0].y, 2);
  EXPECT_EQ(x_vert_planes[0].cloud_seg_map->points[0].z, 3);
}

TEST_F(TestPlane, AssociatePlanes) {
  int matched_plane = this->testAssociatePlanes();
  EXPECT_EQ(matched_plane, 1);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
