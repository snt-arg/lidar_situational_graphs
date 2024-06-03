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
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/rooms.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
using namespace s_graphs;
typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGBNormal PointNormal;

class TestRoom : public ::testing::Test {
  void SetUp() override {
    node = rclcpp::Node::make_shared("test_node");
    node->declare_parameter("room_information", 0.1);
    node->declare_parameter("room_dist_threshold", 1.0);
    node->declare_parameter("dupl_plane_matching_information", 0.01);
    node->declare_parameter("use_parallel_plane_constraint", false);
    node->declare_parameter("use_perpendicular_plane_constraint", false);
    node->declare_parameter("g2o_solver_type", "lm_var_cholmod");

    graph_slam = std::make_shared<s_graphs::GraphSLAM>("lm_var_cholmod");
    finite_room_mapper = std::make_shared<s_graphs::FiniteRoomMapper>(node);
  }

 public:
  void testLookupRooms() {
    situational_graphs_msgs::msg::RoomData room_data;
    situational_graphs_msgs::msg::PlaneData x1_plane_data, x2_plane_data, y1_plane_data,
        y2_plane_data;
    x1_plane_data.id = 1;
    x1_plane_data.nx = 1;
    x1_plane_data.ny = 0;
    x1_plane_data.nz = 0;
    x1_plane_data.d = 10;

    x2_plane_data.id = 2;
    x2_plane_data.nx = -1;
    x2_plane_data.ny = 0;
    x2_plane_data.nz = 0;
    x2_plane_data.d = 4;

    y1_plane_data.id = 3;
    y1_plane_data.nx = 0;
    y1_plane_data.ny = 1;
    y1_plane_data.nz = 0;
    y1_plane_data.d = 6;

    y2_plane_data.id = 4;
    y2_plane_data.nx = 0;
    y2_plane_data.ny = -1;
    y2_plane_data.nz = 0;
    y2_plane_data.d = 4;

    room_data.x_planes.push_back(x1_plane_data);
    room_data.x_planes.push_back(x2_plane_data);
    room_data.y_planes.push_back(y1_plane_data);
    room_data.y_planes.push_back(y2_plane_data);

    room_data.room_center = PlaneUtils::room_center(
        x1_plane_data, x2_plane_data, y1_plane_data, y2_plane_data);

    this->add_keyframe_node();
    this->add_plane_nodes();
    int current_room_id;
    finite_room_mapper->lookup_rooms(graph_slam,
                                     room_data,
                                     x_vert_planes,
                                     y_vert_planes,
                                     dupl_x_vert_planes,
                                     dupl_y_vert_planes,
                                     x_infinite_rooms,
                                     y_infinite_rooms,
                                     rooms_vec,
                                     current_room_id);
  }

  void add_keyframe_node() {
    Eigen::Isometry3d odom;
    odom.setIdentity();
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    keyframe =
        std::make_shared<s_graphs::KeyFrame>(rclcpp::Clock().now(), odom, 0.0, cloud);
    g2o::VertexSE3* v_se3 = new g2o::VertexSE3();
    v_se3->setId(10);
    v_se3->setEstimate(keyframe->odom);
    keyframe->node = v_se3;
  }

  void add_plane_nodes() {
    s_graphs::VerticalPlanes x1_vert_plane, x2_vert_plane;
    s_graphs::VerticalPlanes y1_vert_plane, y2_vert_plane;
    Eigen::Vector4d plane_coeffs;

    x1_vert_plane.id = 1;
    plane_coeffs << 1, 0, 0, 10;
    x1_vert_plane.plane_node = graph_slam->add_plane_node(plane_coeffs);

    x2_vert_plane.id = 2;
    plane_coeffs << -1, 0, 0, 4;
    x2_vert_plane.plane_node = graph_slam->add_plane_node(plane_coeffs);

    x_vert_planes.insert({x1_vert_plane.id, x1_vert_plane});
    x_vert_planes.insert({x2_vert_plane.id, x2_vert_plane});

    y1_vert_plane.id = 3;
    plane_coeffs << 0, 1, 0, 6;
    y1_vert_plane.plane_node = graph_slam->add_plane_node(plane_coeffs);

    y2_vert_plane.id = 4;
    plane_coeffs << 0, -1, 0, 4;
    y2_vert_plane.plane_node = graph_slam->add_plane_node(plane_coeffs);

    y_vert_planes.insert({y1_vert_plane.id, y1_vert_plane});
    y_vert_planes.insert({y2_vert_plane.id, y2_vert_plane});
  }

 public:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<s_graphs::GraphSLAM> graph_slam;
  s_graphs::KeyFrame::Ptr keyframe;
  std::shared_ptr<s_graphs::FiniteRoomMapper> finite_room_mapper;
  std::unordered_map<int, s_graphs::VerticalPlanes> x_vert_planes;
  std::unordered_map<int, s_graphs::VerticalPlanes> y_vert_planes;
  std::deque<std::pair<s_graphs::VerticalPlanes, s_graphs::VerticalPlanes>>
      dupl_x_vert_planes, dupl_y_vert_planes;
  std::unordered_map<int, s_graphs::InfiniteRooms> x_infinite_rooms, y_infinite_rooms;
  std::unordered_map<int, s_graphs::Rooms> rooms_vec;
  // s_graphs::KeyFrame::Ptr keyframe;
};

/* struct TestKeyframe {
  int id_;
  TestKeyframe(int id = 1) : id_(id) {}
  int id() { return id_; }
};

void testFunction() {
  s_graphs::Rooms room;
  room.id = 1;
  std::vector<TestKeyframe*> v;
  v.emplace_back(new TestKeyframe(1));
  v.emplace_back(new TestKeyframe(2));
  v.emplace_back(new TestKeyframe(3));
  generateRoomPointcloud(room, v.begin(), v.end());
} */

TEST_F(TestRoom, TestRoomCentre) {
  this->testLookupRooms();
  auto centre_gt = rooms_vec[0].node->estimate();
  auto global_planes = obtain_global_planes_from_room(
      this->rooms_vec[0], this->x_vert_planes, this->y_vert_planes);
  auto centre = obtain_global_centre_of_room(global_planes);
  if (!centre.has_value()) {
    ASSERT_TRUE(false);
  }
  auto centre_est = centre.value();
  ASSERT_DOUBLE_EQ(centre_gt.translation()(0), centre_est.translation()(0));
  ASSERT_DOUBLE_EQ(centre_gt.translation()(1), centre_est.translation()(1));
  // testFunction();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
