#ifndef __ROOM_UTILS_HPP_
#define __ROOM_UTILS_HPP_
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
#include <g2o/edge_wall_two_planes.hpp>
#include <g2o/vertex_infinite_room.hpp>
#include <g2o/vertex_room.hpp>
#include <g2o/vertex_wall.hpp>
#include <iostream>
#include <s_graphs/floor_mapper.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/graph_visualizer.hpp>
#include <s_graphs/infinite_rooms.hpp>
#include <s_graphs/information_matrix_calculator.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/keyframe_mapper.hpp>
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
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "graph_manager_msgs/msg/attribute.hpp"
#include "graph_manager_msgs/msg/edge.hpp"
#include "graph_manager_msgs/msg/graph.hpp"
#include "graph_manager_msgs/msg/graph_keyframes.hpp"
#include "graph_manager_msgs/msg/node.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/plane_data.hpp"
#include "s_graphs/msg/planes_data.hpp"
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct PlaneGlobalRep {
  Eigen::Vector3d normal;
  Eigen::Vector3d point;
};

std::vector<const s_graphs::VerticalPlanes*> obtain_planes_from_room(
    const s_graphs::Rooms& room,
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes);

bool is_SE3_inside_a_room(const Eigen::Isometry3d& pose,
                          const std::vector<PlaneGlobalRep>& planes);

std::optional<Eigen::Vector3d> find_intersection(const Eigen::Vector3d& point1,
                                                 const Eigen::Vector3d& direction1,
                                                 const Eigen::Vector3d& point2,
                                                 const Eigen::Vector3d& direction2);

std::optional<Eigen::Isometry3d> obtain_global_centre_of_room(
    const std::vector<PlaneGlobalRep>& planes);

std::set<g2o::VertexSE3*> publish_room_keyframes_ids(
    const s_graphs::Rooms& room,
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes);

std::set<g2o::VertexSE3*> filter_inside_room_keyframes(
    const s_graphs::Rooms& room,
    const std::set<g2o::VertexSE3*>& keyframes_candidates);

std::vector<PlaneGlobalRep> obtain_global_planes_from_room(
    const s_graphs::Rooms& room,
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes);

#endif
