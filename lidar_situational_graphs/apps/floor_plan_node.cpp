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
#include "s_graphs/common/ros_utils.hpp"
#include "situational_graphs_msgs/msg/floor_data.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/point_clouds.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "situational_graphs_reasoning_msgs/msg/graph_keyframes.hpp"
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
    new_k_added = false;
    prev_z_diff = 0.0;
    floor_height = -1;
    CURRENT_STATUS = STATE::ON_FLOOR;
    stair_keyframes.clear();
    tmp_stair_keyframes.clear();

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
    map_planes_sub =
        this->create_subscription<situational_graphs_msgs::msg::PlanesData>(
            "s_graphs/all_map_planes",
            100,
            std::bind(&FloorPlanNode::map_planes_callback, this, std::placeholders::_1),
            sub_opt);
    graph_keyframes_sub = this->create_subscription<
        situational_graphs_reasoning_msgs::msg::GraphKeyframes>(
        "s_graphs/graph_keyframes",
        1,
        std::bind(
            &FloorPlanNode::graph_keyframes_callback, this, std::placeholders::_1),
        sub_opt);

    callback_group_publisher =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = callback_group_publisher;

    all_rooms_data_pub =
        this->create_publisher<situational_graphs_msgs::msg::RoomsData>(
            "floor_plan/all_rooms_data", 1, pub_opt);
    floor_data_pub = this->create_publisher<situational_graphs_msgs::msg::FloorData>(
        "floor_plan/floor_data", 10, pub_opt);

    callback_group_floor_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    floor_plan_timer =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&FloorPlanNode::floor_plan_callback, this),
                          callback_group_floor_timer);

    callback_group_keyframe_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    floor_change_det_timer =
        create_wall_timer(std::chrono::milliseconds(100),
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
  void map_planes_callback(
      const situational_graphs_msgs::msg::PlanesData::SharedPtr map_planes_msg) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    x_vert_plane_queue.push_back(map_planes_msg->x_planes);
    y_vert_plane_queue.push_back(map_planes_msg->y_planes);
  }

  void graph_keyframes_callback(
      const situational_graphs_reasoning_msgs::msg::GraphKeyframes::SharedPtr
          graph_keyframes_msg) {
    for (const auto& graph_keyframe_msg : graph_keyframes_msg->keyframes) {
      auto current_keyframe = ROS2Keyframe(graph_keyframe_msg);

      auto found_keyframe = keyframes.find(current_keyframe.id());
      if (found_keyframe == keyframes.end()) {
        keyframe_mutex.lock();
        if (floor_height == -1) floor_height = current_keyframe.node->estimate()(2, 3);
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

  void flush_map_planes(
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
      std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes) {
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
    std::vector<situational_graphs_msgs::msg::PlaneData> current_x_vert_planes,
        current_y_vert_planes;
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
      const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
      const std::vector<situational_graphs_msgs::msg::PlaneData>&
          current_y_vert_planes) {
    std::vector<situational_graphs_msgs::msg::RoomData> room_candidates_vec;
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

      situational_graphs_msgs::msg::RoomsData room_candidates_msg;
      room_candidates_msg.header.stamp = this->now();
      room_candidates_msg.rooms = room_candidates_vec;
      all_rooms_data_pub->publish(room_candidates_msg);
    }
  }

  void extract_floors(
      const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
      const std::vector<situational_graphs_msgs::msg::PlaneData>&
          current_y_vert_planes) {
    std::vector<situational_graphs_msgs::msg::PlaneData> floor_plane_candidates_vec;
    floor_analyzer->perform_floor_segmentation(
        current_x_vert_planes, current_y_vert_planes, floor_plane_candidates_vec);

    geometry_msgs::msg::Pose floor_center;

    if (floor_plane_candidates_vec.size() == 4) {
      floor_center = PlaneUtils::room_center(floor_plane_candidates_vec[0],
                                             floor_plane_candidates_vec[1],
                                             floor_plane_candidates_vec[2],
                                             floor_plane_candidates_vec[3]);

      if (CURRENT_STATUS == ON_FLOOR && !keyframes.empty()) {
        situational_graphs_msgs::msg::FloorData floor_data_msg;
        floor_data_msg.header.stamp = this->now();
        // floor_data_msg.id = floor_level;
        floor_data_msg.floor_center = floor_center;
        // publish floor height as the first captured k height of that floor
        floor_data_msg.state = situational_graphs_msgs::msg::FloorData::ON_FLOOR;
        floor_data_msg.floor_center.position.z = floor_height;
        floor_data_msg.keyframe_ids.clear();
        floor_data_pub->publish(floor_data_msg);
      }
    }
  }

  void floor_change_det_callback() {
    if (!new_k_added || keyframes.size() < 2) return;
    auto current_k = keyframes.rbegin();

    double slope = 0;
    tmp_stair_keyframes.push_back(current_k->second);
    if (tmp_stair_keyframes.size() > 2) {
      slope = linear_regression(tmp_stair_keyframes);
      tmp_stair_keyframes.pop_front();
    }

    switch (CURRENT_STATUS) {
      case STATE::ON_FLOOR: {
        if (slope > 0.5) {
          for (const auto& t_kf : tmp_stair_keyframes) stair_keyframes.push_back(t_kf);
          CURRENT_STATUS = STATE::ASCENDING;
        } else if (slope < -0.5) {
          for (const auto& t_kf : tmp_stair_keyframes) stair_keyframes.push_back(t_kf);
          CURRENT_STATUS = STATE::DESCENDING;
        }

        break;
      }

      case STATE::ASCENDING: {
        if (slope < 0.1) {
          stair_keyframes.push_back(tmp_stair_keyframes.back());
          filter_stairkeyframes(stair_keyframes);
          publish_floor_keyframe_info(
              stair_keyframes.front()->node->estimate()(2, 3) +
              fabs(stair_keyframes.back()->node->estimate()(2, 3) -
                   stair_keyframes.front()->node->estimate()(2, 3)));
          break;
        } else {
          stair_keyframes.push_back(tmp_stair_keyframes.back());
          publish_floor_status();
        }

        break;
      }

      case STATE::DESCENDING: {
        if (slope > -0.1) {
          stair_keyframes.push_back(tmp_stair_keyframes.back());
          filter_stairkeyframes(stair_keyframes);
          publish_floor_keyframe_info(
              stair_keyframes.front()->node->estimate()(2, 3) +
              fabs(stair_keyframes.back()->node->estimate()(2, 3) -
                   stair_keyframes.front()->node->estimate()(2, 3)));
          break;
        } else {
          stair_keyframes.push_back(tmp_stair_keyframes.back());
          publish_floor_status();
        }

        break;
      }

      default:
        break;
    }
    new_k_added = false;
  }

  double linear_regression(const std::deque<KeyFrame::Ptr>& tmp_keyframes) {
    double slope = linear_line_model(tmp_keyframes);
    std::cout << "slope using eigen method: " << slope << std::endl;
    return slope;
  }

  void filter_stairkeyframes(std::deque<KeyFrame::Ptr>& tmp_stair_keyframes) {
    // filter keyframes outside slope
    tmp_stair_keyframes.pop_back();
    tmp_stair_keyframes.pop_back();
  }

  double linear_line_model(const std::deque<KeyFrame::Ptr>& tmp_keyframes) {
    Eigen::VectorXd ids(tmp_keyframes.size());
    Eigen::VectorXd heights(tmp_keyframes.size());

    int id = 0;
    for (const auto& kf : tmp_keyframes) {
      ids(id) = id;
      heights(id) = kf->node->estimate().translation()(2);
      id++;
    }

    Eigen::VectorXd ones = Eigen::VectorXd::Ones(ids.size());
    Eigen::MatrixXd A(ids.size(), 2);
    A << ids, ones;
    Eigen::VectorXd result = A.colPivHouseholderQr().solve(heights);
    return result[0];
  }

  static double compute_height_change(const KeyFrame::Ptr current_k,
                                      const KeyFrame::Ptr prev_k) {
    return (current_k->node->estimate().translation() -
            prev_k->node->estimate().translation())(2);
  }

  double compute_position_change(const KeyFrame::Ptr current_k,
                                 const KeyFrame::Ptr prev_k) {
    return (current_k->node->estimate().translation() -
            prev_k->node->estimate().translation())
        .head(3)
        .norm();
  }

  void publish_floor_keyframe_info(double current_keyframe_height) {
    // publish all the keyframe ids on stairs
    std::vector<int> stair_kd_ids;
    for (const auto& keyframe : stair_keyframes) {
      situational_graphs_reasoning_msgs::msg::Keyframe keyframe_msg;
      stair_kd_ids.push_back(keyframe->id());
    }

    publish_floor_center(stair_kd_ids, current_keyframe_height);
    stair_keyframes.clear();
    CURRENT_STATUS = STATE::ON_FLOOR;
  }

  void publish_floor_center(const std::vector<int>& kf_ids,
                            double current_keyframe_height) {
    std::cout << "publish floor center " << std::endl;
    situational_graphs_msgs::msg::FloorData floor_data_msg;
    floor_data_msg.header.stamp = this->now();
    floor_data_msg.state = situational_graphs_msgs::msg::FloorData::ON_FLOOR;
    floor_data_msg.floor_center.position.x = 0;
    floor_data_msg.floor_center.position.y = 0;
    // publish floor height as the first captured k height of that floor
    floor_data_msg.floor_center.position.z = current_keyframe_height;
    std::cout << "x: " << floor_data_msg.floor_center.position.x
              << ", y: " << floor_data_msg.floor_center.position.y
              << ", z: " << floor_data_msg.floor_center.position.z << std::endl;
    std::cout << "kf ids size: " << kf_ids.size() << std::endl;
    floor_data_msg.keyframe_ids = kf_ids;
    floor_data_pub->publish(floor_data_msg);
    floor_height = current_keyframe_height;
  }

  void publish_floor_status() {
    std::cout << "publish floor status " << std::endl;
    situational_graphs_msgs::msg::FloorData floor_data_msg;
    floor_data_msg.state = situational_graphs_msgs::msg::FloorData::ON_STAIRS;
    floor_data_msg.header.stamp = this->now();
    floor_data_msg.keyframe_ids.clear();
    floor_data_pub->publish(floor_data_msg);
  }

 private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_graph_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::PlanesData>::SharedPtr
      map_planes_sub;
  rclcpp::Subscription<situational_graphs_reasoning_msgs::msg::GraphKeyframes>::
      SharedPtr graph_keyframes_sub;

  rclcpp::Publisher<situational_graphs_msgs::msg::RoomsData>::SharedPtr
      all_rooms_data_pub;

  rclcpp::Publisher<situational_graphs_msgs::msg::FloorData>::SharedPtr floor_data_pub;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::CallbackGroup::SharedPtr callback_group_publisher;

  rclcpp::CallbackGroup::SharedPtr callback_group_floor_timer;
  rclcpp::CallbackGroup::SharedPtr callback_group_keyframe_timer;

 private:
  enum STATE {
    ASCENDING = 0,
    DESCENDING = 1,
    ON_FLOOR = 2,
    FLOOR_CHANGE = 3
  } CURRENT_STATUS;

  rclcpp::TimerBase::SharedPtr floor_plan_timer;
  rclcpp::TimerBase::SharedPtr floor_change_det_timer;
  int accum_keyframes;
  bool save_timings;
  std::ofstream time_recorder;

 private:
  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::unique_ptr<FloorAnalyzer> floor_analyzer;
  std::map<int, s_graphs::KeyFrame::Ptr> keyframes;
  std::deque<KeyFrame::Ptr> stair_keyframes;
  std::deque<KeyFrame::Ptr> tmp_stair_keyframes;
  int floor_level;

  bool new_k_added;
  double prev_z_diff;
  double floor_height;
  std::mutex map_plane_mutex;
  std::mutex keyframe_mutex;
  std::deque<std::vector<situational_graphs_msgs::msg::PlaneData>> x_vert_plane_queue,
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
