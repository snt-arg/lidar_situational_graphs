// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_PUBLISHER_HPP
#define GRAPH_PUBLISHER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <s_graphs/ros_utils.hpp>
#include <s_graphs/ros_time_hash.hpp>
#include <s_graphs/PointClouds.h>
#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>

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
#include <s_graphs/graph_visualizer.hpp>
#include <s_graphs/keyframe_mapper.hpp>
#include <s_graphs/graph_publisher.hpp>
// #include <revit_wall_mapper.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/vertex_room.hpp>
#include <g2o/vertex_wall.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_wall_two_planes.hpp>

#include <unordered_map>
#include <ros1_graph_manager_interface/Attribute.h>
#include <ros1_graph_manager_interface/Edge.h>
#include <ros1_graph_manager_interface/Node.h>
#include <ros1_graph_manager_interface/Graph.h>
class GraphPublisher {
public:
  GraphPublisher(const ros::NodeHandle& private_nh);
  ~GraphPublisher();

public:
  ros1_graph_manager_interface::Graph publish_graph(const g2o::SparseOptimizer* local_graph, std::string graph_type, const std::vector<s_graphs::VerticalPlanes>& x_vert_planes_prior, const std::vector<s_graphs::VerticalPlanes>& y_vert_planes_prior, const std::vector<s_graphs::Rooms>& rooms_vec_prior, const std::vector<s_graphs::VerticalPlanes>& x_vert_planes, const std::vector<s_graphs::VerticalPlanes>& y_vert_planes, const std::vector<s_graphs::Rooms>& rooms_vec, const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms, const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms);

private:
};

#endif  // GRAPH_PUBLISHER_HPP