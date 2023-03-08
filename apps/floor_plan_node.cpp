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

#include <math.h>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <s_graphs/floor_analyzer.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/room_analyzer.hpp>
#include <string>

#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/plane_data.hpp"
#include "s_graphs/msg/planes_data.hpp"
#include "s_graphs/msg/point_clouds.hpp"
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.h"

namespace s_graphs {

class FloorPlanNode : public rclcpp::Node {
 public:
  FloorPlanNode() : Node("floor_segmentation_node") {
    this->initialize_params();
    this->init_ros();
  }

 private:
  void initialize_params() {
    this->declare_parameter("vertex_neigh_thres", 2);
    this->declare_parameter("save_timings", false);

    room_analyzer_params params{
        this->get_parameter("vertex_neigh_thres").get_parameter_value().get<int>()};

    save_timings =
        this->get_parameter("save_timings").get_parameter_value().get<bool>();

    plane_utils.reset(new PlaneUtils());
    room_analyzer.reset(new RoomAnalyzer(params, plane_utils));
    floor_analyzer.reset(new FloorAnalyzer(plane_utils));

    if (save_timings) {
      time_recorder.open("/tmp/floor_seg_computation_time.txt");
      time_recorder << "#time \n";
      time_recorder.close();
    }
  }

  void init_ros() {
    skeleton_graph_sub =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/voxblox_skeletonizer/sparse_graph",
            1,
            std::bind(
                &FloorPlanNode::skeleton_graph_callback, this, std::placeholders::_1));
    map_planes_sub = this->create_subscription<s_graphs::msg::PlanesData>(
        "/s_graphs/all_map_planes",
        100,
        std::bind(&FloorPlanNode::map_planes_callback, this, std::placeholders::_1));

    all_rooms_data_pub = this->create_publisher<s_graphs::msg::RoomsData>(
        "/floor_plan/all_rooms_data", 1);
    floor_data_pub =
        this->create_publisher<s_graphs::msg::RoomData>("/floor_plan/floor_data", 1);

    floor_plane_timer = create_wall_timer(
        std::chrono::seconds(10), std::bind(&FloorPlanNode::floor_plan_callback, this));
  }

  template <typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room
   * candidates
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(
      const visualization_msgs::msg::MarkerArray::SharedPtr skeleton_graph_msg) {
    room_analyzer->analyze_skeleton_graph(skeleton_graph_msg);
  }

  /**
   * @brief get all the mapped planes from all the keyframes
   *
   */
  void map_planes_callback(const s_graphs::msg::PlanesData::SharedPtr map_planes_msg) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    x_vert_plane_queue.push_back(map_planes_msg->x_planes);
    y_vert_plane_queue.push_back(map_planes_msg->y_planes);
  }

  void flush_map_planes(std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes,
                        std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    for (const auto& x_map_planes_msg : x_vert_plane_queue) {
      for (const auto& x_map_plane : x_map_planes_msg) {
        if (!contains(current_x_vert_planes, x_map_plane) ||
            current_x_vert_planes.empty()) {
          current_x_vert_planes.push_back(x_map_plane);
        } else {
          continue;
        }
      }
      x_vert_plane_queue.pop_front();
    }

    for (const auto& y_map_planes_msg : y_vert_plane_queue) {
      for (const auto& y_map_plane : y_map_planes_msg) {
        if (!contains(current_y_vert_planes, y_map_plane) ||
            current_y_vert_planes.empty()) {
          current_y_vert_planes.push_back(y_map_plane);
        } else {
          continue;
        }
      }
      y_vert_plane_queue.pop_front();
    }
  }

  void floor_plan_callback() {
    std::vector<s_graphs::msg::PlaneData> current_x_vert_planes, current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if (current_x_vert_planes.empty() && current_y_vert_planes.empty()) {
      // RCLCPP_INFO(this->get_logger(), "Did not receive any mapped planes");
      return;
    }

    auto t1 = this->now();
    // extract_rooms(current_x_vert_planes, current_y_vert_planes);
    extract_floors(current_x_vert_planes, current_y_vert_planes);
    auto t2 = this->now();
    // std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 -
    // t1).toSec() << std::endl;
    if (save_timings) {
      time_recorder.open("/tmp/floor_seg_computation_time.txt",
                         std::ofstream::out | std::ofstream::app);
      time_recorder << std::to_string((t2 - t1).seconds()) + " \n";
      time_recorder.close();
    }
  }

  void extract_rooms(
      const std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes,
      const std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes) {
    int room_cluster_counter = 0;
    std::vector<s_graphs::msg::RoomData> room_candidates_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters =
        room_analyzer->extract_cloud_clusters();
    std::vector<std::pair<int, int>> connected_subgraph_map =
        room_analyzer->extract_connected_graph();

    for (const auto& cloud_cluster : curr_cloud_clusters) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(
          new pcl::PointCloud<pcl::PointXYZRGB>);

      if (cloud_cluster->points.size() < 10) continue;

      float hull_area;
      room_analyzer->extract_convex_hull(cloud_cluster, cloud_hull, hull_area);
      if (hull_area < 1.5) {
        std::cout << "subgraph area too small to be a room " << std::endl;
        continue;
      }

      RoomInfo room_info = {
          current_x_vert_planes, current_y_vert_planes, cloud_cluster};
      room_analyzer->perform_room_segmentation(room_info,
                                               room_cluster_counter,
                                               cloud_cluster,
                                               room_candidates_vec,
                                               connected_subgraph_map);

      s_graphs::msg::RoomsData room_candidates_msg;
      room_candidates_msg.header.stamp = this->now();
      room_candidates_msg.rooms = room_candidates_vec;
      all_rooms_data_pub->publish(room_candidates_msg);
    }
  }

  void extract_floors(
      const std::vector<s_graphs::msg::PlaneData>& current_x_vert_planes,
      const std::vector<s_graphs::msg::PlaneData>& current_y_vert_planes) {
    std::vector<s_graphs::msg::PlaneData> floor_plane_candidates_vec;
    floor_analyzer->perform_floor_segmentation(
        current_x_vert_planes, current_y_vert_planes, floor_plane_candidates_vec);

    geometry_msgs::msg::Point floor_center;
    if (floor_plane_candidates_vec.size() == 4) {
      floor_center = plane_utils->room_center(floor_plane_candidates_vec[0],
                                              floor_plane_candidates_vec[1],
                                              floor_plane_candidates_vec[2],
                                              floor_plane_candidates_vec[3]);

      s_graphs::msg::RoomData floor_data_msg;
      floor_data_msg.header.stamp = this->now();
      floor_data_msg.id = 0;
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[0]);
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[1]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[2]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[3]);
      floor_data_msg.room_center = floor_center;
      floor_data_pub->publish(floor_data_msg);
    }
  }

 private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_graph_sub;
  rclcpp::Subscription<s_graphs::msg::PlanesData>::SharedPtr map_planes_sub;
  rclcpp::Publisher<s_graphs::msg::RoomsData>::SharedPtr all_rooms_data_pub;
  rclcpp::Publisher<s_graphs::msg::RoomData>::SharedPtr floor_data_pub;

 private:
  rclcpp::TimerBase::SharedPtr floor_plane_timer;
  bool save_timings;
  std::ofstream time_recorder;

 private:
  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::unique_ptr<FloorAnalyzer> floor_analyzer;
  std::shared_ptr<PlaneUtils> plane_utils;

  std::mutex map_plane_mutex;
  std::deque<std::vector<s_graphs::msg::PlaneData>> x_vert_plane_queue,
      y_vert_plane_queue;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::FloorPlanNode>());
  rclcpp::shutdown();
  return 0;
}
