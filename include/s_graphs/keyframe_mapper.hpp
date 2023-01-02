// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_MAPPER_HPP
#define KEYFRAME_MAPPER_HPP

#include <ros/ros.h>
#include <s_graphs/ros_time_hash.hpp>

#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/keyframe_updater.hpp>
#include <s_graphs/loop_detector.hpp>
#include <s_graphs/information_matrix_calculator.hpp>
#include <s_graphs/map_cloud_generator.hpp>
#include <s_graphs/nmea_sentence_parser.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/room_mapper.hpp>
#include <s_graphs/floor_mapper.hpp>
#include <s_graphs/plane_mapper.hpp>
#include <s_graphs/neighbour_mapper.hpp>
#include <s_graphs/plane_analyzer.hpp>

namespace s_graphs {

/**
 * @brief
 */
class KeyframeMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  /**
   * @brief Constructor for class KeyframeMapper
   *
   * @param private_nh
   */
  KeyframeMapper(const ros::NodeHandle& private_nh);
  ~KeyframeMapper();

public:
  /**
   * @brief
   *
   * @param graph_slam
   * @param odom2map
   * @param keyframe_queue
   * @param keyframes
   * @param new_keyframes
   * @param anchor_node
   * @param anchor_edge
   * @param keyframe_hash
   * @return
   */
  int map_keyframes(std::shared_ptr<GraphSLAM>& graph_slam, Eigen::Isometry3d odom2map, std::deque<KeyFrame::Ptr>& keyframe_queue, std::vector<KeyFrame::Ptr>& keyframes, std::deque<KeyFrame::Ptr>& new_keyframes, g2o::VertexSE3*& anchor_node, g2o::EdgeSE3*& anchor_edge, std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash>& keyframe_hash);

private:
  ros::NodeHandle nh;
  int max_keyframes_per_update;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};
}  // namespace s_graphs

#endif  // KEYFRAME_MAPPER_HPP
