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

// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <ctime>
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
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <s_graphs/floor_mapper.hpp>
#include <s_graphs/floors.hpp>
#include <s_graphs/graph_publisher.hpp>
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
#include <s_graphs/room_utils.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/ros_time_hash.hpp>
#include <s_graphs/ros_utils.hpp>
#include <unordered_map>

#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "s_graphs/msg/floor_coeffs.hpp"
#include "s_graphs/msg/plane_data.hpp"
#include "s_graphs/msg/planes_data.hpp"
#include "s_graphs/msg/point_clouds.hpp"
#include "s_graphs/msg/room_data.hpp"
#include "s_graphs/msg/rooms_data.hpp"
#include "s_graphs/srv/dump_graph.hpp"
#include "s_graphs/srv/save_map.hpp"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.h"

namespace s_graphs {

class SGraphsNode : public rclcpp::Node {
 public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZRGBNormal PointNormal;

  SGraphsNode() : Node("s_graphs_node") {
    // init ros parameters
    this->declare_ros_params();
    map_frame_id =
        this->get_parameter("map_frame_id").get_parameter_value().get<std::string>();
    odom_frame_id =
        this->get_parameter("odom_frame_id").get_parameter_value().get<std::string>();

    std::string ns = this->get_namespace();
    if (ns.length()) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      map_frame_id = ns_prefix + "/" + map_frame_id;
      odom_frame_id = ns_prefix + "/" + odom_frame_id;
    }

    map_cloud_resolution =
        this->get_parameter("map_cloud_resolution").get_parameter_value().get<double>();
    wait_trans_odom2map =
        this->get_parameter("wait_trans_odom2map").get_parameter_value().get<bool>();
    got_trans_odom2map = false;
    trans_odom2map.setIdentity();
    odom_path_vec.clear();

    max_keyframes_per_update = this->get_parameter("max_keyframes_per_update")
                                   .get_parameter_value()
                                   .get<int>();
    gps_time_offset =
        this->get_parameter("gps_time_offset").get_parameter_value().get<int>();
    gps_edge_stddev_xy =
        this->get_parameter("gps_edge_stddev_xy").get_parameter_value().get<double>();
    gps_edge_stddev_z =
        this->get_parameter("gps_edge_stddev_z").get_parameter_value().get<double>();
    floor_edge_stddev =
        this->get_parameter("floor_edge_stddev").get_parameter_value().get<double>();

    imu_time_offset =
        this->get_parameter("imu_time_offset").get_parameter_value().get<double>();
    enable_imu_orientation =
        this->get_parameter("enable_imu_orientation").get_parameter_value().get<bool>();
    enable_imu_acceleration = this->get_parameter("enable_imu_acceleration")
                                  .get_parameter_value()
                                  .get<bool>();
    imu_orientation_edge_stddev = this->get_parameter("imu_orientation_edge_stddev")
                                      .get_parameter_value()
                                      .get<double>();
    imu_acceleration_edge_stddev = this->get_parameter("imu_acceleration_edge_stddev")
                                       .get_parameter_value()
                                       .get<double>();

    keyframe_window_size =
        this->get_parameter("keyframe_window_size").get_parameter_value().get<int>();
    extract_planar_surfaces = this->get_parameter("extract_planar_surfaces")
                                  .get_parameter_value()
                                  .get<bool>();
    constant_covariance =
        this->get_parameter("constant_covariance").get_parameter_value().get<bool>();

    infinite_room_information = this->get_parameter("infinite_room_information")
                                    .get_parameter_value()
                                    .get<double>();
    room_information =
        this->get_parameter("room_information").get_parameter_value().get<double>();

    points_topic =
        this->get_parameter("points_topic").get_parameter_value().get<std::string>();

    // tfs
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    odom2map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // subscribers
    init_odom2map_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "odom2map/initial_pose",
        1,
        std::bind(
            &SGraphsNode::init_map2odom_pose_callback, this, std::placeholders::_1));
    while (wait_trans_odom2map && !got_trans_odom2map) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for the Initial Transform between odom and map frame");
      rclcpp::spin_some(shared_from_this());
      usleep(1e6);
    }

    odom_sub.subscribe(this, "odom");
    cloud_sub.subscribe(this, "filtered_points");
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(
        ApproxSyncPolicy(32), odom_sub, cloud_sub));
    sync->registerCallback(&SGraphsNode::cloud_callback, this);

    raw_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        1,
        std::bind(&SGraphsNode::raw_odom_callback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "gpsimu_driver/imu_data",
        1024,
        std::bind(&SGraphsNode::imu_callback, this, std::placeholders::_1));
    floor_sub = this->create_subscription<s_graphs::msg::FloorCoeffs>(
        "floor_detection/floor_coeffs",
        1024,
        std::bind(&SGraphsNode::floor_coeffs_callback, this, std::placeholders::_1));
    room_data_sub = this->create_subscription<s_graphs::msg::RoomsData>(
        "room_segmentation/room_data",
        1,
        std::bind(&SGraphsNode::room_data_callback, this, std::placeholders::_1));
    all_room_data_sub = this->create_subscription<s_graphs::msg::RoomsData>(
        "floor_plan/all_rooms_data",
        1,
        std::bind(&SGraphsNode::all_room_data_callback, this, std::placeholders::_1));
    floor_data_sub = this->create_subscription<s_graphs::msg::RoomData>(
        "floor_plan/floor_data",
        1,
        std::bind(&SGraphsNode::floor_data_callback, this, std::placeholders::_1));

    if (this->get_parameter("enable_gps").get_parameter_value().get<bool>()) {
      gps_sub = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
          "gps/geopoint",
          1024,
          std::bind(&SGraphsNode::gps_callback, this, std::placeholders::_1));
      nmea_sub = this->create_subscription<nmea_msgs::msg::Sentence>(
          "gpsimu_driver/nmea_sentence",
          1024,
          std::bind(&SGraphsNode::nmea_callback, this, std::placeholders::_1));
      navsat_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "gps/navsat",
          1024,
          std::bind(&SGraphsNode::navsat_callback, this, std::placeholders::_1));
    }
    // publishers
    markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "s_graphs/markers", 16);
    odom2map_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "s_graphs/odom2map", 16);
    odom_pose_corrected_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "s_graphs/odom_pose_corrected", 10);
    odom_path_corrected_pub =
        this->create_publisher<nav_msgs::msg::Path>("s_graphs/odom_path_corrected", 10);

    map_points_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("s_graphs/map_points", 1);
    keyframe_map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "s_graphs/keyframe_map_points", 1);
    map_planes_pub =
        this->create_publisher<s_graphs::msg::PlanesData>("s_graphs/map_planes", 1);
    all_map_planes_pub =
        this->create_publisher<s_graphs::msg::PlanesData>("s_graphs/all_map_planes", 1);
    read_until_pub =
        this->create_publisher<std_msgs::msg::Header>("s_graphs/read_until", 32);
    graph_pub = this->create_publisher<graph_manager_msgs::msg::Graph>(
        "s_graphs/graph_structure", 32);
    graph_keyframes_pub =
        this->create_publisher<graph_manager_msgs::msg::GraphKeyframes>(
            "s_graphs/graph_keyframes", 32);
    graph_room_keyframe_pub =
        this->create_publisher<graph_manager_msgs::msg::RoomKeyframe>(
            "s_graphs/graph_room_keyframes", 32);

    dump_service_server = this->create_service<s_graphs::srv::DumpGraph>(
        "s_graphs/dump",
        std::bind(&SGraphsNode::dump_service,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));
    save_map_service_server = this->create_service<s_graphs::srv::SaveMap>(
        "s_graphs/save_map",
        std::bind(&SGraphsNode::save_map_service,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    graph_updated = false;
    prev_edge_count, curr_edge_count = 0;

    double graph_update_interval = this->get_parameter("graph_update_interval")
                                       .get_parameter_value()
                                       .get<double>();
    double keyframe_timer_update_interval =
        this->get_parameter("keyframe_timer_update_interval")
            .get_parameter_value()
            .get<double>();
    double map_cloud_update_interval = this->get_parameter("map_cloud_update_interval")
                                           .get_parameter_value()
                                           .get<double>();

    optimization_timer = this->create_wall_timer(
        std::chrono::seconds(int(graph_update_interval)),
        std::bind(&SGraphsNode::optimization_timer_callback, this));
    keyframe_timer = this->create_wall_timer(
        std::chrono::seconds(int(keyframe_timer_update_interval)),
        std::bind(&SGraphsNode::keyframe_update_timer_callback, this));

    map_publish_timer = this->create_wall_timer(
        std::chrono::seconds(int(map_cloud_update_interval)),
        std::bind(&SGraphsNode::map_points_publish_timer_callback, this));
    graph_publish_timer = this->create_wall_timer(
        std::chrono::seconds(int(graph_update_interval)),
        std::bind(&SGraphsNode::graph_publisher_timer_callback, this));

    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    // one time timer to initialize the classes with the current node obj
    main_timer = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&SGraphsNode::init_subclass, this));
  }

 private:
  void declare_ros_params() {
    this->declare_parameter("map_frame_id", "map");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("map_cloud_resolution", 0.05);
    this->declare_parameter("wait_trans_odom2map", false);
    this->declare_parameter("color_r", 0.0);
    this->declare_parameter("color_g", 0.0);
    this->declare_parameter("color_b", 0.0);
    this->declare_parameter("save_timings", false);

    this->declare_parameter("max_keyframes_per_update", 10);
    this->declare_parameter("gps_time_offset", 10);
    this->declare_parameter("gps_edge_stddev_xy", 10000.0);
    this->declare_parameter("gps_edge_stddev_z", 10.0);
    this->declare_parameter("floor_edge_stddev", 10.0);

    this->declare_parameter("imu_time_offset", 0.0);
    this->declare_parameter("enable_imu_orientation", false);
    this->declare_parameter("enable_imu_acceleration", false);
    this->declare_parameter("imu_orientation_edge_stddev", 0.1);
    this->declare_parameter("imu_acceleration_edge_stddev", 3.0);

    this->declare_parameter("distance_thresh", 5.0);
    this->declare_parameter("accum_distance_thresh", 8.0);
    this->declare_parameter("min_edge_interval", 5.0);
    this->declare_parameter("fitness_score_max_range",
                            std::numeric_limits<double>::max());
    this->declare_parameter("fitness_score_thresh", 0.5);
    this->declare_parameter("registration_method", "NDT_OMP");
    this->declare_parameter("reg_num_threads", 0);
    this->declare_parameter("reg_transformation_epsilon", 0.01);
    this->declare_parameter("reg_maximum_iterations", 64);
    this->declare_parameter("reg_max_correspondence_distance", 2.5);
    this->declare_parameter("reg_correspondence_randomness", 20);
    this->declare_parameter("reg_resolution", 1.0);
    this->declare_parameter("reg_use_reciprocal_correspondences", false);
    this->declare_parameter("reg_max_optimizer_iterations", 20);
    this->declare_parameter("reg_nn_search_method", "DIRECT7");

    this->declare_parameter("use_const_inf_matrix", false);
    this->declare_parameter("const_stddev_x", 0.5);
    this->declare_parameter("const_stddev_q", 0.1);

    this->declare_parameter("var_gain_a", 20.0);
    this->declare_parameter("min_stddev_x", 0.1);
    this->declare_parameter("max_stddev_x", 5.0);
    this->declare_parameter("min_stddev_q", 0.05);
    this->declare_parameter("max_stddev_q", 0.2);

    this->declare_parameter("keyframe_delta_trans", 2.0);
    this->declare_parameter("keyframe_delta_angle", 2.0);
    this->declare_parameter("keyframe_window_size", 1);
    this->declare_parameter("fix_first_node_adaptive", true);

    this->declare_parameter("extract_planar_surfaces", true);
    this->declare_parameter("constant_covariance", true);
    this->declare_parameter("use_parallel_plane_constraint", false);
    this->declare_parameter("use_perpendicular_plane_constraint", false);
    this->declare_parameter("g2o_solver_num_iterations", 1024);
    this->declare_parameter("g2o_solver_type", "lm_var");

    this->declare_parameter("min_seg_points", 100);
    this->declare_parameter("min_horizontal_inliers", 500);
    this->declare_parameter("min_vertical_inliers", 100);
    this->declare_parameter("use_euclidean_filter", true);
    this->declare_parameter("use_shadow_filter", false);
    this->declare_parameter("plane_extraction_frame_id", "base_link");
    this->declare_parameter("plane_visualization_frame_id", "base_link_elevated");

    this->declare_parameter("use_point_to_plane", false);
    this->declare_parameter("plane_information", 0.01);
    this->declare_parameter("plane_dist_threshold", 0.15);
    this->declare_parameter("plane_points_dist", 0.5);
    this->declare_parameter("min_plane_points", 100);

    this->declare_parameter("infinite_room_information", 0.01);
    this->declare_parameter("infinite_room_dist_threshold", 1.0);
    this->declare_parameter("room_information", 0.01);
    this->declare_parameter("room_dist_threshold", 1.0);
    this->declare_parameter("dupl_plane_matching_information", 0.01);

    this->declare_parameter("points_topic", "velodyne_points");
    this->declare_parameter("enable_gps", false);
    this->declare_parameter("graph_update_interval", 3.0);
    this->declare_parameter("keyframe_timer_update_interval", 3.0);
    this->declare_parameter("map_cloud_update_interval", 3.0);
  }

  void init_subclass() {
    graph_slam = std::make_shared<GraphSLAM>(
        this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
        this->get_parameter("save_timings").get_parameter_value().get<bool>());
    keyframe_updater = std::make_unique<KeyframeUpdater>(shared_from_this());
    plane_analyzer = std::make_unique<PlaneAnalyzer>(shared_from_this());
    loop_detector = std::make_unique<LoopDetector>(shared_from_this());
    map_cloud_generator = std::make_unique<MapCloudGenerator>();
    inf_calclator = std::make_unique<InformationMatrixCalculator>(shared_from_this());
    nmea_parser = std::make_unique<NmeaSentenceParser>();
    plane_utils = std::make_unique<PlaneUtils>();
    plane_mapper = std::make_unique<PlaneMapper>(shared_from_this());
    inf_room_mapper = std::make_unique<InfiniteRoomMapper>(shared_from_this());
    finite_room_mapper = std::make_unique<FiniteRoomMapper>(shared_from_this());
    floor_mapper = std::make_unique<FloorMapper>();
    graph_visualizer = std::make_unique<GraphVisualizer>(shared_from_this());
    keyframe_mapper = std::make_unique<KeyframeMapper>(shared_from_this());
    graph_publisher = std::make_unique<GraphPublisher>();
    main_timer->cancel();
  }

 private:
  /**
   * @brief receive the raw odom msg to publish the corrected odom after s
   *
   */
  void raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    Eigen::Isometry3d odom = odom2isometry(odom_msg);
    Eigen::Matrix4f odom_corrected = trans_odom2map * odom.matrix().cast<float>();

    geometry_msgs::msg::PoseStamped pose_stamped_corrected =
        matrix2PoseStamped(odom_msg->header.stamp, odom_corrected, map_frame_id);
    publish_corrected_odom(pose_stamped_corrected);

    // this is dirty but temp solution for no /clock topic in ros2
    geometry_msgs::msg::TransformStamped odom2map_transform = matrix2transform(
        odom_msg->header.stamp, trans_odom2map, map_frame_id, odom_frame_id);
    odom2map_broadcaster->sendTransform(odom2map_transform);
  }

  /**
   * @brief receive the initial transform between map and odom frame
   * @param map2odom_pose_msg
   */
  void init_map2odom_pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    if (got_trans_odom2map) return;

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose_msg->pose.orientation.w,
                                              pose_msg->pose.orientation.x,
                                              pose_msg->pose.orientation.y,
                                              pose_msg->pose.orientation.z)
                               .toRotationMatrix();

    trans_odom2map.block<3, 3>(0, 0) = mat3;
    trans_odom2map(0, 3) = pose_msg->pose.position.x;
    trans_odom2map(1, 3) = pose_msg->pose.position.y;
    trans_odom2map(2, 3) = pose_msg->pose.position.z;

    if (trans_odom2map.isIdentity())
      return;
    else {
      got_trans_odom2map = true;
    }
  }

  void floor_data_callback(const s_graphs::msg::RoomData::SharedPtr floor_data_msg) {
    std::lock_guard<std::mutex> lock(floor_data_mutex);
    floor_data_queue.push_back(*floor_data_msg);
  }

  void flush_floor_data_queue() {
    std::lock_guard<std::mutex> lock(floor_data_mutex);

    if (keyframes.empty()) {
      return;
    } else if (floor_data_queue.empty()) {
      // std::cout << "floor data queue is empty" << std::endl;
      return;
    }
    for (const auto& floor_data_msg : floor_data_queue) {
      floor_mapper->lookup_floors(graph_slam,
                                  floor_data_msg,
                                  floors_vec,
                                  rooms_vec,
                                  x_infinite_rooms,
                                  y_infinite_rooms);

      floor_data_queue.pop_front();
    }
  }

  /**
   * @brief get the room data from room segmentation module
   *
   */
  void room_data_callback(const s_graphs::msg::RoomsData::SharedPtr rooms_msg) {
    std::lock_guard<std::mutex> lock(room_data_queue_mutex);
    room_data_queue.push_back(*rooms_msg);
    // std::cout << "pre_room_data_vec size :" << pre_room_data_vec.size() << std::endl;
  }

  /**
   * @brief flush the room data from room data queue
   *
   */
  void flush_room_data_queue() {
    if (keyframes.empty()) {
      return;
    } else if (room_data_queue.empty()) {
      // std::cout << "room data queue is empty" << std::endl;
      return;
    }

    for (const auto& room_data_msg : room_data_queue) {
      for (const auto& room_data : room_data_msg.rooms) {
        // float dist_robot_room = sqrt(pow(room_data.room_center.x -
        // latest_keyframe->node->estimate().matrix()(0,3),2) +
        // pow(room_data.room_center.y -
        // latest_keyframe->node->estimate().matrix()(1,3),2)); std::cout << "dist robot
        // room: " << dist_robot_room << std::endl;
        if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 2) {
          finite_room_mapper->lookup_rooms(graph_slam,
                                           room_data,
                                           x_vert_planes,
                                           y_vert_planes,
                                           dupl_x_vert_planes,
                                           dupl_y_vert_planes,
                                           x_infinite_rooms,
                                           y_infinite_rooms,
                                           rooms_vec);
        }
        // x infinite_room
        else if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 0) {
          inf_room_mapper->lookup_infinite_rooms(graph_slam,
                                                 PlaneUtils::plane_class::X_VERT_PLANE,
                                                 room_data,
                                                 x_vert_planes,
                                                 y_vert_planes,
                                                 dupl_x_vert_planes,
                                                 dupl_y_vert_planes,
                                                 x_infinite_rooms,
                                                 y_infinite_rooms,
                                                 rooms_vec);
        }
        // y infinite_room
        else if (room_data.x_planes.size() == 0 && room_data.y_planes.size() == 2) {
          inf_room_mapper->lookup_infinite_rooms(graph_slam,
                                                 PlaneUtils::plane_class::Y_VERT_PLANE,
                                                 room_data,
                                                 x_vert_planes,
                                                 y_vert_planes,
                                                 dupl_x_vert_planes,
                                                 dupl_y_vert_planes,
                                                 x_infinite_rooms,
                                                 y_infinite_rooms,
                                                 rooms_vec);
        }
      }

      room_data_queue_mutex.lock();
      room_data_queue.pop_front();
      room_data_queue_mutex.unlock();
    }
  }

  /**
   * @brief get the entire room data from floor plan module to detect neighbours
   *
   */
  void all_room_data_callback(const s_graphs::msg::RoomsData::SharedPtr rooms_msg) {
    std::lock_guard<std::mutex> lock(all_room_data_queue_mutex);
    all_room_data_queue.push_back(*rooms_msg);
  }

  void flush_all_room_data_queue() {
    std::lock_guard<std::mutex> lock(all_room_data_queue_mutex);

    if (keyframes.empty()) {
      return;
    } else if (all_room_data_queue.empty()) {
      std::cout << "all room data queue is empty" << std::endl;
      return;
    }

    for (const auto& room_data_msg : all_room_data_queue) {
      // neighbour_mapper->detect_room_neighbours(graph_slam, room_data_msg,
      // x_infinite_rooms, y_infinite_rooms, rooms_vec);
      // neighbour_mapper->factor_room_neighbours(graph_slam, room_data_msg,
      // x_infinite_rooms, y_infinite_rooms, rooms_vec);

      all_room_data_queue.pop_front();
    }
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg,
                      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    const rclcpp::Time& stamp = cloud_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if (!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if (keyframe_queue.empty()) {
        std_msgs::msg::Header read_until;
        read_until.stamp = stamp + rclcpp::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub->publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub->publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
   * (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if (keyframe_queue.empty()) {
      // std::cout << "keyframe_queue is empty " << std::endl;
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = keyframe_mapper->map_keyframes(graph_slam,
                                                       odom2map,
                                                       keyframe_queue,
                                                       keyframes,
                                                       new_keyframes,
                                                       anchor_node,
                                                       anchor_edge,
                                                       keyframe_hash);

    // perform planar segmentation
    for (int i = 0; i < new_keyframes.size(); i++) {
      // perform planar segmentation
      if (extract_planar_surfaces) {
        std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec =
            plane_analyzer->extract_segmented_planes(new_keyframes[i]->cloud);
        plane_mapper->map_extracted_planes(graph_slam,
                                           new_keyframes[i],
                                           extracted_cloud_vec,
                                           x_vert_planes,
                                           y_vert_planes,
                                           hort_planes);
      }
    }
    std_msgs::msg::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + rclcpp::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub->publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub->publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(),
                         keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if (grmc.status != 'A') {
      return;
    }

    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(
        new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg) {
    geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(
        new geographic_msgs::msg::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;

    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    rclcpp::Time(gps_msg->header.stamp) += rclcpp::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if (keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for (auto& keyframe : keyframes) {
      if (keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
      }

      if (keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for (auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt =
            (rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*gps)->header.stamp) - keyframe->stamp).seconds();
        if (std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if (0.2 < std::abs((rclcpp::Time((*closest_gps)->header.stamp) - keyframe->stamp)
                             .seconds())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM
      // coordinate
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

      // the first gps data position will be the origin of the map
      if (!zero_utm) {
        zero_utm = xyz;
      }
      xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if (std::isnan(xyz.z())) {
        Eigen::Matrix2d information_matrix =
            Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se3_prior_xy_edge(
            keyframe->node, xyz.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        edge =
            graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);

      updated = true;
    }

    auto remove_loc = std::upper_bound(
        gps_queue.begin(),
        gps_queue.end(),
        keyframes.back()->stamp,
        [=](const rclcpp::Time& stamp,
            const geographic_msgs::msg::GeoPointStamped::SharedPtr& geopoint) {
          return stamp < geopoint->header.stamp;
        });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    if (!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    rclcpp::Time(imu_msg->header.stamp) += rclcpp::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if (keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for (auto& keyframe : keyframes) {
      if (keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }

      if (keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for (auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt =
            (rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp).seconds();
        auto dt2 = (rclcpp::Time((*imu)->header.stamp) - keyframe->stamp).seconds();
        if (std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if (0.2 < std::abs((rclcpp::Time((*closest_imu)->header.stamp) - keyframe->stamp)
                             .seconds())) {
        continue;
      }

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::msg::Vector3Stamped acc_imu;
      geometry_msgs::msg::Vector3Stamped acc_base;
      geometry_msgs::msg::QuaternionStamped quat_imu;
      geometry_msgs::msg::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id =
          (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = rclcpp::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;

      try {
        tf_buffer->transform(acc_imu, acc_base, base_frame_id);
        tf_buffer->transform(quat_imu, quat_base, base_frame_id);
      } catch (std::exception& e) {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }

      keyframe->acceleration =
          Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w,
                                                 quat_base.quaternion.x,
                                                 quat_base.quaternion.y,
                                                 quat_base.quaternion.z);
      keyframe->orientation = keyframe->orientation;
      if (keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if (enable_imu_orientation) {
        Eigen::MatrixXd info =
            Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(
            keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }

      if (enable_imu_acceleration) {
        Eigen::MatrixXd info =
            Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(
            keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(
        imu_queue.begin(),
        imu_queue.end(),
        keyframes.back()->stamp,
        [=](const rclcpp::Time& stamp, const sensor_msgs::msg::Imu::SharedPtr imu) {
          return stamp < imu->header.stamp;
        });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(
      const s_graphs::msg::FloorCoeffs::SharedPtr floor_coeffs_msg) {
    if (floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered
   * keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if (keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for (const auto& floor_coeffs : floor_coeffs_queue) {
      if (rclcpp::Time(floor_coeffs->header.stamp) > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(rclcpp::Time(floor_coeffs->header.stamp));
      if (found == keyframe_hash.end()) {
        continue;
      }

      if (!floor_plane_node) {
        floor_plane_node =
            graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0],
                             floor_coeffs->coeffs[1],
                             floor_coeffs->coeffs[2],
                             floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information =
          Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(
          keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc =
        std::upper_bound(floor_coeffs_queue.begin(),
                         floor_coeffs_queue.end(),
                         latest_keyframe_stamp,
                         [=](const rclcpp::Time& stamp,
                             const s_graphs::msg::FloorCoeffs::SharedPtr coeffs) {
                           return stamp < coeffs->header.stamp;
                         });
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate graph structure and publish it
   * @param event
   */
  void graph_publisher_timer_callback() {
    g2o::SparseOptimizer* local_graph;
    graph_mutex.lock();
    local_graph = graph_slam->graph.get();
    graph_mutex.unlock();
    std::string graph_type;
    if (std::string("/robot1") == this->get_namespace()) {
      graph_type = "Prior";
    } else {
      graph_type = "Online";
    }
    auto graph_structure = graph_publisher->publish_graph(local_graph,
                                                          "Online",
                                                          x_vert_planes_prior,
                                                          y_vert_planes_prior,
                                                          rooms_vec_prior,
                                                          x_vert_planes,
                                                          y_vert_planes,
                                                          rooms_vec,
                                                          x_infinite_rooms,
                                                          y_infinite_rooms);
    graph_structure.name = graph_type;

    auto graph_keyframes =
        graph_publisher->publish_graph_keyframes(local_graph, this->keyframes);
    graph_pub->publish(graph_structure);
    graph_keyframes_pub->publish(graph_keyframes);

    static RoomsKeyframeGenerator keyframe_generator(
        &this->x_vert_planes, &this->y_vert_planes, &this->keyframes);

    for (auto room : this->rooms_vec) {
      keyframe_generator.addRoom(room);
    }
    for (auto room : keyframe_generator.getExtendedRooms()) {
      graph_room_keyframe_pub->publish(convertExtendedRoomToRosMsg(room));
    }
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback() {
    if (map_points_pub->get_subscription_count() < 0 || !graph_updated) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if (!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(
        new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    g2o::SparseOptimizer* local_graph;
    graph_mutex.lock();
    local_graph = graph_snapshot;
    graph_mutex.unlock();

    std::vector<VerticalPlanes> x_plane_snapshot, y_plane_snapshot;
    vert_plane_snapshot_mutex.lock();
    x_plane_snapshot = x_vert_planes_snapshot;
    y_plane_snapshot = y_vert_planes_snapshot;
    vert_plane_snapshot_mutex.unlock();

    std::vector<HorizontalPlanes> hort_plane_snapshot;
    hort_plane_snapshot_mutex.lock();
    hort_plane_snapshot = hort_planes_snapshot;
    hort_plane_snapshot_mutex.unlock();

    std::vector<InfiniteRooms> x_infinite_room_snapshot, y_infinite_room_snapshot;
    infinite_room_snapshot_mutex.lock();
    x_infinite_room_snapshot = x_infinite_rooms_snapshot;
    y_infinite_room_snapshot = y_infinite_rooms_snapshot;
    infinite_room_snapshot_mutex.unlock();

    std::vector<Rooms> room_snapshot;
    room_snapshot_mutex.lock();
    room_snapshot = rooms_vec_snapshot;
    room_snapshot_mutex.unlock();

    auto markers = graph_visualizer->create_marker_array(
        this->now(),
        local_graph,
        x_plane_snapshot,
        y_plane_snapshot,
        hort_plane_snapshot,
        x_infinite_room_snapshot,
        y_infinite_room_snapshot,
        room_snapshot,
        loop_detector->get_distance_thresh() * 2.0,
        keyframes,
        floors_vec);
    markers_pub->publish(markers);

    publish_all_mapped_planes(x_plane_snapshot, y_plane_snapshot);
    map_points_pub->publish(*cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then
   * optimizes the pose graph
   * @param event
   */
  void keyframe_update_timer_callback() {
    // add keyframes and floor coeffs in the queues to the pose graph
    keyframes_mutex.lock();
    bool keyframe_updated = flush_keyframe_queue();
    keyframes_mutex.unlock();

    if (!keyframe_updated) {
      std_msgs::msg::Header read_until;
      read_until.stamp = this->now() + rclcpp::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub->publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub->publish(read_until);
    }

    if (!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() &
        !flush_imu_queue()) {
      return;
    }

    // publish mapped planes
    publish_mapped_planes(x_vert_planes, y_vert_planes);

    // flush the room poses from room detector and no need to return if no rooms found
    flush_room_data_queue();

    // flush the floor poses from the floor planner and no need to return if no floors
    // found
    flush_floor_data_queue();

    // flush all the rooms queue to map neighbours
    // flush_all_room_data_queue();

    // loop detection
    std::vector<Loop::Ptr> loops =
        loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for (const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(
          loop->key1->cloud, loop->key2->cloud, relpose);
      auto edge = graph_slam->add_se3_edge(
          loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }

    std::copy(
        new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node
    // pose so the first node moves freely while trying to stay around the origin
    if (anchor_node && this->get_parameter("fix_first_node_adaptive")
                           .get_parameter_value()
                           .get<bool>()) {
      Eigen::Isometry3d anchor_target =
          static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    vert_plane_snapshot_mutex.lock();
    x_vert_planes_snapshot = x_vert_planes;
    y_vert_planes_snapshot = y_vert_planes;
    vert_plane_snapshot_mutex.unlock();

    hort_plane_snapshot_mutex.lock();
    hort_planes_snapshot = hort_planes;
    hort_plane_snapshot_mutex.unlock();

    infinite_room_snapshot_mutex.lock();
    x_infinite_rooms_snapshot = x_infinite_rooms;
    y_infinite_rooms_snapshot = y_infinite_rooms;
    infinite_room_snapshot_mutex.unlock();

    room_snapshot_mutex.lock();
    rooms_vec_snapshot = rooms_vec;
    room_snapshot_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(
        keyframes.begin(),
        keyframes.end(),
        snapshot.begin(),
        [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then
   * optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback() {
    if (keyframes.empty()) return;

    graph_mutex.lock();
    std::shared_ptr<GraphSLAM> local_graph = graph_slam;
    graph_mutex.unlock();

    curr_edge_count = local_graph->retrive_total_nbr_of_edges();
    if (curr_edge_count <= prev_edge_count) return;

    keyframes_mutex.lock();
    const auto& keyframe = keyframes.back();
    keyframes_mutex.unlock();

    // optimize the pose graph
    int num_iterations = this->get_parameter("g2o_solver_num_iterations")
                             .get_parameter_value()
                             .get<int>();

    try {
      if ((local_graph->optimize(num_iterations)) > 0 && !constant_covariance)
        compute_plane_cov();
    } catch (std::invalid_argument& e) {
      std::cout << e.what() << std::endl;
      throw 1;
    }

    // publish tf
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    geometry_msgs::msg::TransformStamped ts = matrix2transform(
        keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub->publish(ts);

    // merge_duplicate_planes();
    graph_mutex.lock();
    graph_snapshot = local_graph->graph.get();
    graph_mutex.unlock();

    graph_updated = true;
    prev_edge_count = curr_edge_count;
  }

  /**
   * @brief publish the pointcloud snapshots information from the last n keyframes
   *
   */
  void publish_keyframe_mapped_points() {
    if (keyframes.empty()) return;

    std::vector<KeyFrame::Ptr> keyframe_window(
        keyframes.end() - std::min<int>(keyframes.size(), keyframe_window_size),
        keyframes.end());

    std::vector<KeyFrameSnapshot::Ptr> snapshot_vec;
    for (std::vector<KeyFrame::Ptr>::reverse_iterator it = keyframe_window.rbegin();
         it != keyframe_window.rend();
         ++it) {
      KeyFrameSnapshot::Ptr snapshot = std::make_shared<KeyFrameSnapshot>(*it);
      snapshot_vec.push_back(snapshot);
    }

    pcl::PointCloud<PointT>::Ptr keyframe_cloud =
        map_cloud_generator->generate(snapshot_vec, 0.0);

    keyframe_cloud->header.frame_id = map_frame_id;
    keyframe_cloud->header.stamp = snapshot_vec.back()->cloud->header.stamp;
    sensor_msgs::msg::PointCloud2 keyframe_cloud_msg;
    pcl::toROSMsg(*keyframe_cloud, keyframe_cloud_msg);
    keyframe_map_points_pub->publish(keyframe_cloud_msg);
  }

  /**
   * @brief publish the mapped plane information from the last n keyframes
   *
   */
  void publish_mapped_planes(std::vector<VerticalPlanes> x_vert_planes_snapshot,
                             std::vector<VerticalPlanes> y_vert_planes_snapshot) {
    if (keyframes.empty()) return;

    std::vector<KeyFrame::Ptr> keyframe_window(
        keyframes.end() - std::min<int>(keyframes.size(), keyframe_window_size),
        keyframes.end());
    std::map<int, int> unique_x_plane_ids, unique_y_plane_ids;
    for (std::vector<KeyFrame::Ptr>::reverse_iterator it = keyframe_window.rbegin();
         it != keyframe_window.rend();
         ++it) {
      for (const auto& x_plane_id : (*it)->x_plane_ids) {
        auto result = unique_x_plane_ids.insert(std::pair<int, int>(x_plane_id, 1));
        // if(result.second == false) std::cout << "x plane already existed with id : "
        // << x_plane_id << std::endl;
      }

      for (const auto& y_plane_id : (*it)->y_plane_ids) {
        auto result = unique_y_plane_ids.insert(std::pair<int, int>(y_plane_id, 1));
        // if(result.second == false) std::cout << "y plane already existed with id : "
        // << y_plane_id << std::endl;
      }
    }

    s_graphs::msg::PlanesData vert_planes_data;
    vert_planes_data.header.stamp = keyframes.back()->stamp;
    for (const auto& unique_x_plane_id : unique_x_plane_ids) {
      auto local_x_vert_plane =
          std::find_if(x_vert_planes_snapshot.begin(),
                       x_vert_planes_snapshot.end(),
                       boost::bind(&VerticalPlanes::id, _1) == unique_x_plane_id.first);
      if (local_x_vert_plane == x_vert_planes_snapshot.end()) continue;
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (*local_x_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (*local_x_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data : (*local_x_vert_plane).cloud_seg_map->points) {
        geometry_msgs::msg::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.x_planes.push_back(plane_data);
    }

    for (const auto& unique_y_plane_id : unique_y_plane_ids) {
      auto local_y_vert_plane =
          std::find_if(y_vert_planes_snapshot.begin(),
                       y_vert_planes_snapshot.end(),
                       boost::bind(&VerticalPlanes::id, _1) == unique_y_plane_id.first);
      if (local_y_vert_plane == y_vert_planes_snapshot.end()) continue;
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (*local_y_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (*local_y_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data : (*local_y_vert_plane).cloud_seg_map->points) {
        geometry_msgs::msg::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.y_planes.push_back(plane_data);
    }
    map_planes_pub->publish(vert_planes_data);
  }

  /**
   * @brief publish all the mapped plane information from the entire set of keyframes
   *
   */
  void publish_all_mapped_planes(
      const std::vector<VerticalPlanes>& x_vert_planes_snapshot,
      const std::vector<VerticalPlanes>& y_vert_planes_snapshot) {
    if (keyframes.empty()) return;

    s_graphs::msg::PlanesData vert_planes_data;
    vert_planes_data.header.stamp = keyframes.back()->stamp;
    for (const auto& x_vert_plane : x_vert_planes_snapshot) {
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (x_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (x_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data : (x_vert_plane).cloud_seg_map->points) {
        geometry_msgs::msg::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.x_planes.push_back(plane_data);
    }

    for (const auto& y_vert_plane : y_vert_planes_snapshot) {
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (y_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (y_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data : (y_vert_plane).cloud_seg_map->points) {
        geometry_msgs::msg::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.y_planes.push_back(plane_data);
    }
    all_map_planes_pub->publish(vert_planes_data);
  }

  /**
   * @brief merge all the duplicate x and y planes detected by room/infinite_rooms
   */
  void merge_duplicate_planes() {
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> curr_dupl_x_vert_planes;
    // check the number of occurances of the same duplicate planes
    for (auto it_1 = dupl_x_vert_planes.begin(); it_1 != dupl_x_vert_planes.end();
         ++it_1) {
      int id_count = 0;
      int current_id = (*it_1).second.id;
      for (auto it_2 = dupl_x_vert_planes.begin(); it_2 != dupl_x_vert_planes.end();
           ++it_2) {
        if (current_id == (*it_2).second.id) {
          id_count++;
        }
      }
      if (id_count > 3) {
        curr_dupl_x_vert_planes.push_back(*it_1);
      }
    }

    for (auto it = curr_dupl_x_vert_planes.begin(); it != curr_dupl_x_vert_planes.end();
         ++it) {
      std::set<g2o::HyperGraph::Edge*> edges = (*it).first.plane_node->edges();

      for (auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(*edge_itr);
        if (edge_se3_plane) {
          /* get the keyframe node and connect it with the original mapped plane node */
          g2o::VertexSE3* keyframe_node =
              dynamic_cast<g2o::VertexSE3*>(edge_se3_plane->vertices()[0]);
          g2o::Plane3D local_plane =
              keyframe_node->estimate().inverse() * (*it).second.plane_node->estimate();
          Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
          auto edge = graph_slam->add_se3_plane_edge(keyframe_node,
                                                     (*it).second.plane_node,
                                                     local_plane.coeffs(),
                                                     information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);

          /* remove the edge between the keyframe and found duplicate plane */
          if (graph_slam->remove_se3_plane_edge(edge_se3_plane))
            std::cout << "removed edge - pose se3 x plane " << std::endl;
          continue;
        }

        g2o::EdgeRoomXPlane* edge_infinite_room_xplane =
            dynamic_cast<g2o::EdgeRoomXPlane*>(*edge_itr);
        if (edge_infinite_room_xplane) {
          /* remove the edge between the infinite_room and the duplicate found plane */
          /* get infinite_room id from the vertex */
          g2o::VertexRoomXYLB* infinite_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(
              edge_infinite_room_xplane->vertices()[0]);
          auto found_x_infinite_room = std::find_if(
              x_infinite_rooms.begin(),
              x_infinite_rooms.end(),
              boost::bind(&InfiniteRooms::id, _1) == infinite_room_node->id());

          if (found_x_infinite_room == x_infinite_rooms.end()) continue;

          /* if any of the mapped plane_id of the infinite_room equal to dupl plane id
           * replace it */
          if ((*found_x_infinite_room).plane1_id == (*it).first.id) {
            (*found_x_infinite_room).plane1_id = (*it).second.id;
            (*found_x_infinite_room).plane1 = (*it).second.plane;
          } else if ((*found_x_infinite_room).plane2_id == (*it).first.id) {
            (*found_x_infinite_room).plane2_id = (*it).second.id;
            (*found_x_infinite_room).plane2 = (*it).second.plane;
          }
          /* Add edge between infinite_room and current mapped plane */
          Eigen::Vector4d found_mapped_plane1_coeffs =
              (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                               found_mapped_plane1_coeffs);
          double meas_plane1 = inf_room_mapper->infinite_room_measurement(
              PlaneUtils::plane_class::X_VERT_PLANE,
              infinite_room_node->estimate(),
              found_mapped_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_infinite_room_plane;
          information_infinite_room_plane(0, 0) = infinite_room_information;
          // information_infinite_room_plane(1, 1) = infinite_room_information;

          auto edge_plane =
              graph_slam->add_room_xplane_edge(infinite_room_node,
                                               (*it).second.plane_node,
                                               meas_plane1,
                                               information_infinite_room_plane);
          graph_slam->add_robust_kernel(edge_plane, "Huber", 1.0);

          if (graph_slam->remove_room_xplane_edge(edge_infinite_room_xplane))
            std::cout << "removed edge - infinite_room xplane " << std::endl;
          continue;
        }
        /* TODO: analyze if connecting room node with (*it).second.plane is necessary */
        g2o::EdgeRoomXPlane* edge_room_xplane =
            dynamic_cast<g2o::EdgeRoomXPlane*>(*edge_itr);
        if (edge_room_xplane) {
          /* remove the edge between the room and the duplicate found plane */
          /* get room id from the vertex */
          g2o::VertexRoomXYLB* room_node =
              dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_xplane->vertices()[0]);
          auto found_room =
              std::find_if(rooms_vec.begin(),
                           rooms_vec.end(),
                           boost::bind(&Rooms::id, _1) == room_node->id());

          if (found_room == rooms_vec.end()) continue;

          if ((*found_room).plane_x1_id == (*it).first.id) {
            (*found_room).plane_x1_id = (*it).second.id;
            (*found_room).plane_x1 = (*it).second.plane;
          } else if ((*found_room).plane_x2_id == (*it).first.id) {
            (*found_room).plane_x2_id = (*it).second.id;
            (*found_room).plane_x2 = (*it).second.plane;
          }

          /* Add edge between room and current mapped plane */
          Eigen::Vector4d found_mapped_x_plane1_coeffs =
              (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
                                               found_mapped_x_plane1_coeffs);
          double x_plane1_meas = finite_room_mapper->room_measurement(
              PlaneUtils::plane_class::X_VERT_PLANE,
              room_node->estimate(),
              found_mapped_x_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_room_plane;
          information_room_plane(0, 0) = room_information;
          // information_room_plane(1, 1) = room_information;
          auto edge_x_plane1 = graph_slam->add_room_xplane_edge(room_node,
                                                                (*it).second.plane_node,
                                                                x_plane1_meas,
                                                                information_room_plane);
          graph_slam->add_robust_kernel(edge_x_plane1, "Huber", 1.0);

          if (graph_slam->remove_room_xplane_edge(edge_room_xplane))
            std::cout << "removed edge - room xplane " << std::endl;
          continue;
        }
      }

      /* finally remove the duplicate plane node */
      if (graph_slam->remove_plane_node((*it).first.plane_node)) {
        auto mapped_plane =
            std::find_if(x_vert_planes.begin(),
                         x_vert_planes.end(),
                         boost::bind(&VerticalPlanes::id, _1) == (*it).first.id);
        x_vert_planes.erase(mapped_plane);
        std::cout << "removed x vert plane " << std::endl;
      }
    }

    // remove only the current detected duplicate planes
    for (int i = 0; i < curr_dupl_x_vert_planes.size(); ++i) {
      for (int j = 0; j < dupl_x_vert_planes.size();) {
        if (curr_dupl_x_vert_planes[i].second.id == dupl_x_vert_planes[j].second.id) {
          dupl_x_vert_planes.erase(dupl_x_vert_planes.begin() + j);
        } else
          ++j;
      }
    }

    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> curr_dupl_y_vert_planes;
    // check the number of occurances of the same duplicate planes
    for (auto it_1 = dupl_y_vert_planes.begin(); it_1 != dupl_y_vert_planes.end();
         ++it_1) {
      int id_count = 0;
      int current_id = (*it_1).second.id;
      for (auto it_2 = dupl_y_vert_planes.begin(); it_2 != dupl_y_vert_planes.end();
           ++it_2) {
        if (current_id == (*it_2).second.id) {
          id_count++;
        }
      }
      if (id_count > 3) {
        curr_dupl_y_vert_planes.push_back(*it_1);
      }
    }

    for (auto it = curr_dupl_y_vert_planes.begin(); it != curr_dupl_y_vert_planes.end();
         ++it) {
      std::set<g2o::HyperGraph::Edge*> edges = (*it).first.plane_node->edges();

      for (auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(*edge_itr);
        if (edge_se3_plane) {
          /* get the keyframe node and connect it with the original mapped plane node */
          g2o::VertexSE3* keyframe_node =
              dynamic_cast<g2o::VertexSE3*>(edge_se3_plane->vertices()[0]);
          g2o::Plane3D local_plane =
              keyframe_node->estimate().inverse() * (*it).second.plane_node->estimate();
          Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
          auto edge = graph_slam->add_se3_plane_edge(keyframe_node,
                                                     (*it).second.plane_node,
                                                     local_plane.coeffs(),
                                                     information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);

          /* remove the edge between the keyframe and found duplicate plane */
          if (graph_slam->remove_se3_plane_edge(edge_se3_plane))
            std::cout << "remove edge - pose se3 yplane " << std::endl;
          continue;
        }
        g2o::EdgeRoomYPlane* edge_infinite_room_yplane =
            dynamic_cast<g2o::EdgeRoomYPlane*>(*edge_itr);
        if (edge_infinite_room_yplane) {
          /* remove the edge between the infinite_room and the duplicate found plane */
          g2o::VertexRoomXYLB* infinite_room_node = dynamic_cast<g2o::VertexRoomXYLB*>(
              edge_infinite_room_yplane->vertices()[0]);
          auto found_y_infinite_room = std::find_if(
              y_infinite_rooms.begin(),
              y_infinite_rooms.end(),
              boost::bind(&InfiniteRooms::id, _1) == infinite_room_node->id());
          if (found_y_infinite_room == y_infinite_rooms.end()) continue;

          if ((*found_y_infinite_room).plane1_id == (*it).first.id) {
            (*found_y_infinite_room).plane1_id = (*it).second.id;
            (*found_y_infinite_room).plane1 = (*it).second.plane;
          } else if ((*found_y_infinite_room).plane2_id == (*it).first.id) {
            (*found_y_infinite_room).plane2_id = (*it).second.id;
            (*found_y_infinite_room).plane2 = (*it).second.plane;
          }

          /* Add edge between infinite_room and current mapped plane */
          Eigen::Vector4d found_mapped_plane1_coeffs =
              (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                               found_mapped_plane1_coeffs);
          double meas_plane1 = inf_room_mapper->infinite_room_measurement(
              PlaneUtils::plane_class::Y_VERT_PLANE,
              infinite_room_node->estimate(),
              found_mapped_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_infinite_room_plane;
          information_infinite_room_plane(0, 0) = infinite_room_information;
          // information_infinite_room_plane(1, 1) = infinite_room_information;

          auto edge_plane =
              graph_slam->add_room_yplane_edge(infinite_room_node,
                                               (*it).second.plane_node,
                                               meas_plane1,
                                               information_infinite_room_plane);
          graph_slam->add_robust_kernel(edge_plane, "Huber", 1.0);

          if (graph_slam->remove_room_yplane_edge(edge_infinite_room_yplane))
            std::cout << "removed edge - infinite_room yplane " << std::endl;
          continue;
        }
        g2o::EdgeRoomYPlane* edge_room_yplane =
            dynamic_cast<g2o::EdgeRoomYPlane*>(*edge_itr);
        if (edge_room_yplane) {
          /* remove the edge between the room and the duplicate found plane */
          g2o::VertexRoomXYLB* room_node =
              dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_yplane->vertices()[0]);
          auto found_room =
              std::find_if(rooms_vec.begin(),
                           rooms_vec.end(),
                           boost::bind(&Rooms::id, _1) == room_node->id());
          if (found_room == rooms_vec.end()) continue;

          if ((*found_room).plane_y1_id == (*it).first.id) {
            (*found_room).plane_y1_id = (*it).second.id;
            (*found_room).plane_y1 = (*it).second.plane;
          } else if ((*found_room).plane_y2_id == (*it).first.id) {
            (*found_room).plane_y2_id = (*it).second.id;
            (*found_room).plane_y2 = (*it).second.plane;
          }

          /* Add edge between room and current mapped plane */
          Eigen::Vector4d found_mapped_y_plane1_coeffs =
              (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
                                               found_mapped_y_plane1_coeffs);
          double y_plane1_meas = finite_room_mapper->room_measurement(
              PlaneUtils::plane_class::Y_VERT_PLANE,
              room_node->estimate(),
              found_mapped_y_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_room_plane;
          information_room_plane(0, 0) = room_information;
          // information_room_plane(1, 1) = room_information;
          auto edge_y_plane1 = graph_slam->add_room_xplane_edge(room_node,
                                                                (*it).second.plane_node,
                                                                y_plane1_meas,
                                                                information_room_plane);
          graph_slam->add_robust_kernel(edge_y_plane1, "Huber", 1.0);

          if (graph_slam->remove_room_yplane_edge(edge_room_yplane))
            std::cout << "removed edge - room yplane " << std::endl;
          continue;
        }
      }
      /* finally remove the duplicate plane node */
      if (graph_slam->remove_plane_node((*it).first.plane_node)) {
        auto mapped_plane =
            std::find_if(y_vert_planes.begin(),
                         y_vert_planes.end(),
                         boost::bind(&VerticalPlanes::id, _1) == (*it).first.id);
        y_vert_planes.erase(mapped_plane);
        std::cout << "removed y vert plane " << std::endl;
      }
    }

    // remove only the current detected duplicate planes
    for (int i = 0; i < curr_dupl_y_vert_planes.size(); ++i) {
      for (int j = 0; j < dupl_y_vert_planes.size();) {
        if (curr_dupl_y_vert_planes[i].second.id == dupl_y_vert_planes[j].second.id) {
          dupl_y_vert_planes.erase(dupl_y_vert_planes.begin() + j);
        } else
          ++j;
      }
    }
  }

  /**
   * @brief publish odom corrected pose and path
   */
  void publish_corrected_odom(geometry_msgs::msg::PoseStamped pose_stamped_corrected) {
    nav_msgs::msg::Path path_stamped_corrected;
    path_stamped_corrected.header = pose_stamped_corrected.header;
    odom_path_vec.push_back(pose_stamped_corrected);
    path_stamped_corrected.poses = odom_path_vec;

    odom_pose_corrected_pub->publish(pose_stamped_corrected);
    odom_path_corrected_pub->publish(path_stamped_corrected);
  }

  /**
   * @brief compute the plane covariances
   */
  void compute_plane_cov() {
    g2o::SparseBlockMatrix<Eigen::MatrixXd> plane_spinv_vec;
    std::vector<std::pair<int, int>> plane_pairs_vec;
    for (int i = 0; i < x_vert_planes.size(); ++i) {
      x_vert_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(
          std::make_pair(x_vert_planes[i].plane_node->hessianIndex(),
                         x_vert_planes[i].plane_node->hessianIndex()));
    }
    for (int i = 0; i < y_vert_planes.size(); ++i) {
      y_vert_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(
          std::make_pair(y_vert_planes[i].plane_node->hessianIndex(),
                         y_vert_planes[i].plane_node->hessianIndex()));
    }
    for (int i = 0; i < hort_planes.size(); ++i) {
      hort_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(
          std::make_pair(hort_planes[i].plane_node->hessianIndex(),
                         hort_planes[i].plane_node->hessianIndex()));
    }

    if (!plane_pairs_vec.empty()) {
      if (graph_slam->compute_landmark_marginals(plane_spinv_vec, plane_pairs_vec)) {
        int i = 0;
        while (i < x_vert_planes.size()) {
          // std::cout << "covariance of x plane " << i << " " <<
          // y_vert_planes[i].covariance << std::endl;
          x_vert_planes[i].covariance =
              plane_spinv_vec
                  .block(x_vert_planes[i].plane_node->hessianIndex(),
                         x_vert_planes[i].plane_node->hessianIndex())
                  ->eval()
                  .cast<double>();
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(x_vert_planes[i].covariance);
          if (lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of x plane not PSD" << i << " " <<
            // x_vert_planes[i].covariance << std::endl;
            x_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
        }
        i = 0;
        while (i < y_vert_planes.size()) {
          y_vert_planes[i].covariance =
              plane_spinv_vec
                  .block(y_vert_planes[i].plane_node->hessianIndex(),
                         y_vert_planes[i].plane_node->hessianIndex())
                  ->eval()
                  .cast<double>();
          // std::cout << "covariance of y plane " << i << " " <<
          // y_vert_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(y_vert_planes[i].covariance);
          if (lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of y plane not PSD " << i << " " <<
            // y_vert_planes[i].covariance << std::endl;
            y_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
        }
        i = 0;
        while (i < hort_planes.size()) {
          hort_planes[i].covariance =
              plane_spinv_vec
                  .block(hort_planes[i].plane_node->hessianIndex(),
                         hort_planes[i].plane_node->hessianIndex())
                  ->eval()
                  .cast<double>();
          // std::cout << "covariance of y plane " << i << " " <<
          // hort_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(hort_planes[i].covariance);
          if (lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of y plane not PSD " << i << " " <<
            // hort_planes[i].covariance << std::endl;
            hort_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
        }
      }
    }
  }

  /**
   * @brief convert the body points of planes to map frame for mapping
   */
  void convert_plane_points_to_map() {
    for (int i = 0; i < x_vert_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(
          new pcl::PointCloud<PointNormal>());

      for (int k = 0; k < x_vert_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose =
            x_vert_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();
        for (size_t j = 0; j < x_vert_planes[i].cloud_seg_body_vec[k]->points.size();
             ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() =
              pose * x_vert_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      x_vert_planes[i].cloud_seg_map = cloud_seg_map;
    }

    for (int i = 0; i < y_vert_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(
          new pcl::PointCloud<PointNormal>());

      for (int k = 0; k < y_vert_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose =
            y_vert_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();

        for (size_t j = 0; j < y_vert_planes[i].cloud_seg_body_vec[k]->points.size();
             ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() =
              pose * y_vert_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      y_vert_planes[i].cloud_seg_map = cloud_seg_map;
    }

    for (int i = 0; i < hort_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(
          new pcl::PointCloud<PointNormal>());

      for (int k = 0; k < hort_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose =
            hort_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();

        for (size_t j = 0; j < hort_planes[i].cloud_seg_body_vec[k]->points.size();
             ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() =
              pose * hort_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      hort_planes[i].cloud_seg_map = cloud_seg_map;
    }
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(const std::shared_ptr<s_graphs::srv::DumpGraph::Request> req,
                    std::shared_ptr<s_graphs::srv::DumpGraph::Response> res) {
    std::lock_guard<std::mutex> lock(graph_mutex);

    std::string directory = req->destination;

    if (directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if (!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for (int i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if (zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() %
                          zero_utm->z()
                   << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id())
        << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id())
        << std::endl;
    ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id())
        << std::endl;

    res->success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(const std::shared_ptr<s_graphs::srv::SaveMap::Request> req,
                        std::shared_ptr<s_graphs::srv::SaveMap::Response> res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req->resolution);
    if (!cloud) {
      res->success = false;
      return true;
    }

    if (zero_utm && req->utm) {
      for (auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if (zero_utm) {
      std::ofstream ofs(req->destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() %
                 zero_utm->z()
          << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req->destination, *cloud);
    res->success = ret == 0;

    return true;
  }

 private:
  // ROS
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr keyframe_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;
  rclcpp::TimerBase::SharedPtr graph_publish_timer;
  rclcpp::TimerBase::SharedPtr map_to_odom_broadcast_timer;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub;
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
  // sensor_msgs::msg::PointCloud2> approximate_policy;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                          sensor_msgs::msg::PointCloud2>
      ApproxSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<s_graphs::msg::FloorCoeffs>::SharedPtr floor_sub;
  rclcpp::Subscription<s_graphs::msg::RoomsData>::SharedPtr room_data_sub;
  rclcpp::Subscription<s_graphs::msg::RoomsData>::SharedPtr all_room_data_sub;
  rclcpp::Subscription<s_graphs::msg::RoomData>::SharedPtr floor_data_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_odom2map_sub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  bool wait_trans_odom2map, got_trans_odom2map;
  std::vector<geometry_msgs::msg::PoseStamped> odom_path_vec;
  std::string map_frame_id;
  std::string odom_frame_id;
  std::string points_topic;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_corrected_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_corrected_pub;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr read_until_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_map_points_pub;
  rclcpp::Publisher<s_graphs::msg::PlanesData>::SharedPtr map_planes_pub;
  rclcpp::Publisher<s_graphs::msg::PlanesData>::SharedPtr all_map_planes_pub;
  rclcpp::Publisher<graph_manager_msgs::msg::Graph>::SharedPtr graph_pub;
  rclcpp::Publisher<graph_manager_msgs::msg::GraphKeyframes>::SharedPtr
      graph_keyframes_pub;
  rclcpp::Publisher<graph_manager_msgs::msg::RoomKeyframe>::SharedPtr
      graph_room_keyframe_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom2map_broadcaster;

  rclcpp::Service<s_graphs::srv::DumpGraph>::SharedPtr dump_service_server;
  rclcpp::Service<s_graphs::srv::SaveMap>::SharedPtr save_map_service_server;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue;

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue;

  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<s_graphs::msg::FloorCoeffs::SharedPtr> floor_coeffs_queue;

  // vertical and horizontal planes
  int keyframe_window_size;
  bool extract_planar_surfaces;
  bool constant_covariance;
  double min_plane_points;
  double infinite_room_information;
  double room_information;
  std::vector<VerticalPlanes> x_vert_planes,
      y_vert_planes;  // vertically segmented planes
  std::vector<VerticalPlanes> x_vert_planes_prior, y_vert_planes_prior;
  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      dupl_y_vert_planes;                     // vertically segmented planes
  std::vector<HorizontalPlanes> hort_planes;  // horizontally segmented planes
  std::vector<InfiniteRooms> x_infinite_rooms,
      y_infinite_rooms;          // infinite_rooms segmented from planes
  std::vector<Rooms> rooms_vec;  // rooms segmented from planes
  std::vector<Rooms> rooms_vec_prior;
  std::vector<Floors> floors_vec;
  int prev_edge_count, curr_edge_count;

  std::mutex vert_plane_snapshot_mutex;
  std::vector<VerticalPlanes> x_vert_planes_snapshot,
      y_vert_planes_snapshot;  // snapshot of vertically segmented planes

  std::mutex hort_plane_snapshot_mutex;
  std::vector<HorizontalPlanes> hort_planes_snapshot;

  std::mutex infinite_room_snapshot_mutex;
  std::vector<InfiniteRooms> x_infinite_rooms_snapshot, y_infinite_rooms_snapshot;

  std::mutex room_snapshot_mutex;
  std::vector<Rooms> rooms_vec_snapshot;

  // Seg map queue
  std::mutex cloud_seg_mutex;
  std::deque<s_graphs::msg::PointClouds::Ptr> clouds_seg_queue;

  // room data queue
  std::mutex room_data_queue_mutex;
  std::deque<s_graphs::msg::RoomsData> room_data_queue;

  // all room data queue
  std::mutex all_room_data_queue_mutex;
  std::deque<s_graphs::msg::RoomsData> all_room_data_queue;

  std::mutex floor_data_mutex;
  std::deque<s_graphs::msg::RoomData> floor_data_queue;

  std::mutex keyframes_mutex;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  std::mutex graph_mutex;
  g2o::SparseOptimizer* graph_snapshot;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::shared_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<PlaneAnalyzer> plane_analyzer;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<PlaneUtils> plane_utils;
  std::unique_ptr<PlaneMapper> plane_mapper;
  std::unique_ptr<InfiniteRoomMapper> inf_room_mapper;
  std::unique_ptr<FiniteRoomMapper> finite_room_mapper;
  std::unique_ptr<FloorMapper> floor_mapper;
  std::unique_ptr<GraphVisualizer> graph_visualizer;
  std::unique_ptr<KeyframeMapper> keyframe_mapper;
  std::unique_ptr<GraphPublisher> graph_publisher;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::SGraphsNode>());
  rclcpp::shutdown();
  return 0;
}
