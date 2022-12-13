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

#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>

#include <g2o/vertex_room.hpp>
#include <g2o/vertex_wall.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_point_to_plane.hpp>

#include <g2o/edge_infinite_room_plane.hpp>
#include <g2o/edge_room.hpp>
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
  void convert_graph_to_string(std::unique_ptr<s_graphs::GraphSLAM>& graph_slam);

private:
};

#endif  // GRAPH_PUBLISHER_HPP