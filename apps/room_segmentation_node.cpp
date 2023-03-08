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
#include <pcl/ModelCoefficients.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <boost/format.hpp>
#include <cmath>
#include <iostream>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/room_analyzer.hpp>
#include <string>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

class RoomSegmentationNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZRGBNormal PointT;

  RoomSegmentationNode() : Node("room_segmentation_node") {
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

    if (save_timings) {
      time_recorder.open("/tmp/room_seg_computation_time.txt");
      time_recorder << "#time \n";
      time_recorder.close();
    }
  }

  void init_ros() {
    skeleton_graph_sub =
        this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/voxblox_skeletonizer/sparse_graph",
            1,
            std::bind(&RoomSegmentationNode::skeleton_graph_callback,
                      this,
                      std::placeholders::_1));
    map_planes_sub = this->create_subscription<s_graphs::msg::PlanesData>(
        "/s_graphs/map_planes",
        100,
        std::bind(
            &RoomSegmentationNode::map_planes_callback, this, std::placeholders::_1));

    cluster_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/room_segmentation/cluster_cloud", 1);
    cluster_clouds_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/room_segmentation/cluster_clouds", 1);
    room_data_pub = this->create_publisher<s_graphs::msg::RoomsData>(
        "/room_segmentation/room_data", 1);
    room_centers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/room_segmentation/room_centers", 1);
    refined_skeleton_graph_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/room_segmentation/refined_skeleton_graph", 1);

    room_detection_timer = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RoomSegmentationNode::room_detection_callback, this));
  }

  template <typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  void room_detection_callback() {
    std::vector<s_graphs::msg::PlaneData> current_x_vert_planes, current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if (current_x_vert_planes.empty() && current_y_vert_planes.empty()) {
      // RCLCPP_INFO(this->get_logger(), "Did not receive any mapped planes");
      return;
    }

    auto t1 = this->now();
    extract_rooms(current_x_vert_planes, current_y_vert_planes);
    auto t2 = this->now();
    // std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 -
    // t1).seconds() << std::endl;
    if (save_timings) {
      time_recorder.open("/tmp/room_seg_computation_time.txt",
                         std::ofstream::out | std::ofstream::app);
      time_recorder << std::to_string((t2 - t1).seconds()) + " \n";
      time_recorder.close();
    }
  }

  /**
   * @brief get the vertical planes in map frame
   * @param map_planes_msg
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

  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room
   * candidates
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(
      visualization_msgs::msg::MarkerArray::SharedPtr skeleton_graph_msg) {
    room_analyzer->analyze_skeleton_graph(skeleton_graph_msg);
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud
   *
   */
  void extract_rooms(std::vector<s_graphs::msg::PlaneData> current_x_vert_planes,
                     std::vector<s_graphs::msg::PlaneData> current_y_vert_planes) {
    int room_cluster_counter = 0;
    visualization_msgs::msg::MarkerArray refined_skeleton_marker_array;
    std::vector<s_graphs::msg::RoomData> room_candidates_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visualizer(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_visualizer(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters =
        room_analyzer->extract_cloud_clusters();
    std::vector<std::pair<int, int>> connected_subgraph_map =
        room_analyzer->extract_connected_graph();
    visualization_msgs::msg::MarkerArray skeleton_marker_array =
        room_analyzer->extract_marker_array_clusters();

    int cluster_id = 0;
    for (const auto& cloud_cluster : curr_cloud_clusters) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(
          new pcl::PointCloud<pcl::PointXYZRGB>);

      if (cloud_cluster->points.size() < 10) continue;

      float hull_area;
      room_analyzer->extract_convex_hull(cloud_cluster, cloud_hull, hull_area);
      if (hull_area < 1.5) {
        // std::cout << "subgraph area too small to be a room " << std::endl;
        continue;
      }

      RoomInfo room_info = {
          current_x_vert_planes, current_y_vert_planes, cloud_cluster};
      bool found_room =
          room_analyzer->perform_room_segmentation(room_info,
                                                   room_cluster_counter,
                                                   cloud_cluster,
                                                   room_candidates_vec,
                                                   connected_subgraph_map);
      if (found_room) {
        refined_skeleton_marker_array.markers.push_back(
            skeleton_marker_array.markers[2 * cluster_id]);
        refined_skeleton_marker_array.markers.push_back(
            skeleton_marker_array.markers[2 * cluster_id + 1]);
      }
      for (int i = 0; i < cloud_cluster->points.size(); ++i) {
        cloud_visualizer->points.push_back(cloud_cluster->points[i]);
      }
      cluster_id++;
    }

    s_graphs::msg::RoomsData room_candidates_msg;
    room_candidates_msg.header.stamp = this->now();
    room_candidates_msg.rooms = room_candidates_vec;
    room_data_pub->publish(room_candidates_msg);
    viz_room_centers(room_candidates_msg);

    sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_visualizer, cloud_cluster_msg);
    cloud_cluster_msg.header.stamp = this->now();
    cloud_cluster_msg.header.frame_id = "map";
    cluster_cloud_pub->publish(cloud_cluster_msg);

    refined_skeleton_graph_pub->publish(refined_skeleton_marker_array);
  }

  void viz_room_centers(s_graphs::msg::RoomsData room_vec) {
    visualization_msgs::msg::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = "map";
    room_marker.header.stamp = this->now();
    room_marker.ns = "rooms";
    room_marker.id = 0;
    room_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    room_marker.color.r = 1;
    // room_marker.color.g = 0.07;
    // room_marker.color.b = 0.0;
    // room_marker.color.a = 1;

    for (const auto& room : room_vec.rooms) {
      geometry_msgs::msg::Point point;
      point.x = room.room_center.x;
      point.y = room.room_center.y;
      point.z = 7.0;
      room_marker.points.push_back(point);

      std_msgs::msg::ColorRGBA point_color;
      if (room.x_planes.size() == 2 && room.y_planes.size() == 2) {
        point_color.r = 1;
        point_color.a = 1;
      } else {
        point_color.g = 1;
        point_color.a = 1;
      }
      room_marker.colors.push_back(point_color);
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(room_marker);
    room_centers_pub->publish(markers);
  }

 private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      skeleton_graph_sub;
  rclcpp::Subscription<s_graphs::msg::PlanesData>::SharedPtr map_planes_sub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_clouds_pub;
  rclcpp::Publisher<s_graphs::msg::RoomsData>::SharedPtr room_data_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr room_centers_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      refined_skeleton_graph_pub;

  /* private variables */
 private:
  rclcpp::TimerBase::SharedPtr room_detection_timer;
  std::mutex map_plane_mutex;

  std::vector<s_graphs::msg::PlaneData> x_vert_plane_vec, y_vert_plane_vec;
  std::deque<std::vector<s_graphs::msg::PlaneData>> x_vert_plane_queue,
      y_vert_plane_queue;

  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::shared_ptr<PlaneUtils> plane_utils;
  bool save_timings;
  std::ofstream time_recorder;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::RoomSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}