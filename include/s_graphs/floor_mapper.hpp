// SPDX-License-Identifier: BSD-2-Clause

#ifndef FLOOR_MAPPER_HPP
#define FLOOR_MAPPER_HPP

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>

#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/graph_slam.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/corridors.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/plane_utils.hpp>

#include <g2o/vertex_room.hpp>
#include <g2o/vertex_corridor.hpp>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/edge_corridor_plane.hpp>
#include <g2o/edge_room.hpp>

namespace s_graphs {

class FloorMapper {
  typedef pcl::PointXYZRGBNormal PointNormal;

public:
  FloorMapper(const ros::NodeHandle& private_nh);
  ~FloorMapper();

private:
  ros::NodeHandle nh;
  std::unique_ptr<PlaneUtils> plane_utils;

public:
  void lookup_floors(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec);

private:
  void factor_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomData room_data, std::vector<s_graphs::Floors>& floors_vec);
  void update_floor_node(std::unique_ptr<GraphSLAM>& graph_slam, g2o::VertexRoomXYLB* floor_node, const s_graphs::RoomData room_data);

private:
  void factor_floor_room_nodes();
};

}  // namespace s_graphs

#endif  // FLOOR_MAPPER_HPP
