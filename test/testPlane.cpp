
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/plane_analyzer.hpp>
#include <s_graphs/plane_mapper.hpp>
#include <s_graphs/plane_utils.hpp>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

typedef pcl::PointXYZI PointT;

TEST(testPlane, ConvertPlaneToMap) {
  ros::NodeHandle nh;
  s_graphs::PlaneMapper plane_mapper(nh);
  s_graphs::GraphSLAM graph_slam;
  Eigen::Isometry3d odom;
  odom.setIdentity();

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  s_graphs::KeyFrame::Ptr keyframe(
      new s_graphs::KeyFrame(ros::Time::now(), odom, 0.0, cloud));
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
  EXPECT_EQ(map_plane_vec(3), 10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testPlane");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}