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
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/frontend/floor_analyzer.hpp>
#include <s_graphs/frontend/room_analyzer.hpp>
#include <string>

#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reasoning_msgs/msg/graph_keyframes.hpp"
#include "s_graphs/common/ros_utils.hpp"
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
    floor_level = 0;
    new_k_added = false, on_stairs = false;
    num_k_added = 0;
    delta_diff = 0.0;
    prev_z_diff = 0.0;

    room_analyzer_params params{
        this->get_parameter("vertex_neigh_thres").get_parameter_value().get<int>()};

    save_timings =
        this->get_parameter("save_timings").get_parameter_value().get<bool>();

    room_analyzer.reset(new RoomAnalyzer(params));
    floor_analyzer.reset(new FloorAnalyzer());

    if (save_timings) {
      time_recorder.open("/tmp/floor_seg_computation_time.txt");
      time_recorder << "#time \n";
      time_recorder.close();
    }
  }

  void init_ros() {
    callback_group_subscriber =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber;

    skeleton_graph_sub =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "voxblox_skeletonizer/sparse_graph",
            1,
            std::bind(
                &FloorPlanNode::skeleton_graph_callback, this, std::placeholders::_1),
            sub_opt);
    map_planes_sub = this->create_subscription<s_graphs::msg::PlanesData>(
        "s_graphs/all_map_planes",
        100,
        std::bind(&FloorPlanNode::map_planes_callback, this, std::placeholders::_1),
        sub_opt);
    graph_keyframes_sub =
        this->create_subscription<reasoning_msgs::msg::GraphKeyframes>(
            "s_graphs/graph_keyframes",
            1,
            std::bind(
                &FloorPlanNode::graph_keyframes_callback, this, std::placeholders::_1),
            sub_opt);

    callback_group_publisher =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = callback_group_publisher;

    all_rooms_data_pub = this->create_publisher<s_graphs::msg::RoomsData>(
        "floor_plan/all_rooms_data", 1, pub_opt);
    floor_data_pub = this->create_publisher<s_graphs::msg::RoomData>(
        "floor_plan/floor_data", 1, pub_opt);

    callback_group_floor_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    floor_plan_timer =
        create_wall_timer(std::chrono::seconds(10),
                          std::bind(&FloorPlanNode::floor_plan_callback, this),
                          callback_group_floor_timer);

    callback_group_keyframe_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    floor_change_det_timer =
        create_wall_timer(std::chrono::seconds(1),
                          std::bind(&FloorPlanNode::floor_change_det_callback, this),
                          callback_group_keyframe_timer);
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

  void graph_keyframes_callback(
      const reasoning_msgs::msg::GraphKeyframes::SharedPtr graph_keyframes_msg) {
    for (const auto& graph_keyframe_msg : graph_keyframes_msg->keyframes) {
      auto current_keyframe = ROS2Keyframe(graph_keyframe_msg);

      auto found_keyframe = keyframes.find(current_keyframe.id());
      if (found_keyframe == keyframes.end()) {
        keyframe_mutex.lock();
        keyframes.insert(
            {current_keyframe.id(), std::make_shared<KeyFrame>(current_keyframe)});
        new_k_added = true;
        keyframe_mutex.unlock();
      } else {
        keyframe_mutex.lock();
        found_keyframe->second->node->setEstimate(current_keyframe.node->estimate());
        keyframe_mutex.unlock();
      }
    }

    return;
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
    std::vector<s_graphs::msg::RoomData> room_candidates_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters =
        room_analyzer->extract_cloud_clusters();

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

      visualization_msgs::msg::MarkerArray current_cloud_marker;
      RoomInfo room_info = {
          current_x_vert_planes, current_y_vert_planes, cloud_cluster};
      room_analyzer->perform_room_segmentation(
          room_info, cloud_cluster, room_candidates_vec, current_cloud_marker);

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

    geometry_msgs::msg::Pose floor_center;
    if (floor_plane_candidates_vec.size() == 4) {
      floor_center = PlaneUtils::room_center(floor_plane_candidates_vec[0],
                                             floor_plane_candidates_vec[1],
                                             floor_plane_candidates_vec[2],
                                             floor_plane_candidates_vec[3]);

      s_graphs::msg::RoomData floor_data_msg;
      floor_data_msg.header.stamp = this->now();
      floor_data_msg.id = floor_level;
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[0]);
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[1]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[2]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[3]);
      floor_data_msg.room_center = floor_center;
      floor_data_pub->publish(floor_data_msg);
    }
  }

  void floor_change_det_callback() {
    if (!new_k_added) return;

    // get the pose difference between the last and the second last keyframe
    if (keyframes.size() >= 2 && first_stair_k.empty()) {
      auto current_k = keyframes.rbegin();
      auto prev_k = std::next(current_k);

      prev_z_diff = (current_k->second->node->estimate() *
                     prev_k->second->node->estimate().inverse())(2, 3);

      if (prev_z_diff > 0.5) {
        first_stair_k.push_back(prev_k->second);
      }
    } else if (keyframes.size() >= 2) {
      auto current_k = keyframes.rbegin();
      double current_z_diff = (current_k->second->node->estimate() *
                               first_stair_k[0]->node->estimate().inverse())(2, 3);

      std::cout << "z diff between the keyframe and first_stair_k: " << current_z_diff
                << std::endl;

      delta_diff = current_z_diff - prev_z_diff;
      std::cout << "delta_diff: " << delta_diff << std::endl;
      std::cout << "num_k_added: " << num_k_added << std::endl;

      if (current_z_diff > 1.0 && !on_stairs) {
        std::cout << "climbing stairs " << std::endl;
        on_stairs = true;
        num_k_added = 0;
      } else if (on_stairs && num_k_added < 1) {
        std::cout << "still climbing stairs " << std::endl;
        num_k_added++;
      } else if (on_stairs && num_k_added >= 1) {
        if (fabs(delta_diff) < 0.5) {
          std::cout << "on new floor level" << std::endl;
          on_stairs = false;
          // publish the new floor message with all the relevant kfs information
          publish_floor_keyframe_info();
        } else
          num_k_added++;
      }
      prev_z_diff = current_z_diff;
    }

    new_k_added = false;
  }

  void publish_floor_keyframe_info() {
    // first get all keyframes that belong to stairs

    num_k_added = 0;
    first_stair_k.clear();
  }

 private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_graph_sub;
  rclcpp::Subscription<s_graphs::msg::PlanesData>::SharedPtr map_planes_sub;
  rclcpp::Subscription<reasoning_msgs::msg::GraphKeyframes>::SharedPtr
      graph_keyframes_sub;

  rclcpp::Publisher<s_graphs::msg::RoomsData>::SharedPtr all_rooms_data_pub;
  rclcpp::Publisher<s_graphs::msg::RoomData>::SharedPtr floor_data_pub;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::CallbackGroup::SharedPtr callback_group_publisher;

  rclcpp::CallbackGroup::SharedPtr callback_group_floor_timer;
  rclcpp::CallbackGroup::SharedPtr callback_group_keyframe_timer;

 private:
  rclcpp::TimerBase::SharedPtr floor_plan_timer;
  rclcpp::TimerBase::SharedPtr floor_change_det_timer;
  int accum_keyframes;
  bool save_timings;
  std::ofstream time_recorder;

 private:
  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::unique_ptr<FloorAnalyzer> floor_analyzer;
  std::map<int, s_graphs::KeyFrame::Ptr> keyframes;
  std::vector<KeyFrame::Ptr> first_stair_k;
  int floor_level;

  bool new_k_added, on_stairs;
  int num_k_added;
  double delta_diff;
  double prev_z_diff;
  std::mutex map_plane_mutex;
  std::mutex keyframe_mutex;
  std::deque<std::vector<s_graphs::msg::PlaneData>> x_vert_plane_queue,
      y_vert_plane_queue;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor multi_executor;
  auto node = std::make_shared<s_graphs::FloorPlanNode>();
  multi_executor.add_node(node);
  multi_executor.spin();
  rclcpp::shutdown();
  return 0;
}
