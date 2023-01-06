// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_VISUALIZER_HPP
#define GRAPH_VISUALIZER_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/format.hpp>
#include <cmath>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <iostream>
#include <s_graphs/floor_mapper.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/information_matrix_calculator.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/keyframe_updater.hpp>
#include <s_graphs/loop_detector.hpp>
#include <s_graphs/map_cloud_generator.hpp>
#include <s_graphs/nmea_sentence_parser.hpp>
#include <s_graphs/plane_analyzer.hpp>
#include <s_graphs/plane_mapper.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/room_mapper.hpp>
#include <s_graphs/rooms.hpp>
#include <string>

#include "geometry_msgs/msg/point.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.h"

namespace s_graphs {

/**
 * @brief
 */
class GraphVisualizer {
  typedef pcl::PointXYZRGBNormal PointNormal;

 public:
  /**
   * @brief Constructor for class GraphVisualizer.
   *
   * @param node
   */
  GraphVisualizer(const rclcpp::Node::SharedPtr node);
  ~GraphVisualizer();

 public:
  /**
   * @brief Creates a marker array
   *
   * @param stamp
   * @param local_graph
   * @param x_plane_snapshot
   * @param y_plane_snapshot
   * @param loop_detector_radius
   * @param keyframes
   * @param floors_vec
   * @return A MarkerArray message.
   */
  visualization_msgs::msg::MarkerArray create_marker_array(
      const rclcpp::Time& stamp,
      const g2o::SparseOptimizer* local_graph,
      const std::vector<VerticalPlanes>& x_plane_snapshot,
      const std::vector<VerticalPlanes>& y_plane_snapshot,
      const std::vector<HorizontalPlanes>& hort_plane_snapshot,
      std::vector<InfiniteRooms> x_infinite_room_snapshot,
      std::vector<InfiniteRooms> y_infinite_room_snapshot,
      std::vector<Rooms> room_snapshot,
      double loop_detector_radius,
      std::vector<KeyFrame::Ptr> keyframes,
      std::vector<Floors> floors_vec);

 private:
  std::string map_frame_id;
  double color_r, color_g, color_b;
};

}  // namespace s_graphs

#endif  // GRAPH_VISUALIZER_HPP
