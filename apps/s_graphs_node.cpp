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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Dense>
#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <mutex>
#include <s_graphs/backend/floor_mapper.hpp>
#include <s_graphs/backend/gps_mapper.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/imu_mapper.hpp>
#include <s_graphs/backend/keyframe_mapper.hpp>
#include <s_graphs/backend/loop_mapper.hpp>
#include <s_graphs/backend/plane_mapper.hpp>
#include <s_graphs/backend/room_graph_generator.hpp>
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/backend/wall_mapper.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/graph_utils.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/map_cloud_generator.hpp>
#include <s_graphs/common/nmea_sentence_parser.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/common/ros_time_hash.hpp>
#include <s_graphs/common/ros_utils.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/loop_detector.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <s_graphs/visualization/graph_publisher.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>
#include <unordered_map>

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
#include "s_graphs/msg/wall_data.hpp"
#include "s_graphs/msg/walls_data.hpp"
#include "s_graphs/srv/dump_graph.hpp"
#include "s_graphs/srv/load_graph.hpp"
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
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      map_frame_id = ns_prefix + "/" + map_frame_id;
      odom_frame_id = ns_prefix + "/" + odom_frame_id;
    }

    map_cloud_resolution =
        this->get_parameter("map_cloud_resolution").get_parameter_value().get<double>();
    wait_trans_odom2map =
        this->get_parameter("wait_trans_odom2map").get_parameter_value().get<bool>();
    use_map2map_transform =
        this->get_parameter("use_map2map_transform").get_parameter_value().get<bool>();
    got_trans_odom2map = false;
    trans_odom2map.setIdentity();
    odom_path_vec.clear();

    max_keyframes_per_update = this->get_parameter("max_keyframes_per_update")
                                   .get_parameter_value()
                                   .get<int>();

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

    optimization_window_size = this->get_parameter("optimization_window_size")
                                   .get_parameter_value()
                                   .get<int>();

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

    callback_group_subscriber =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber;

    // subscribers
    init_odom2map_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "odom2map/initial_pose",
        1,
        std::bind(
            &SGraphsNode::init_map2odom_pose_callback, this, std::placeholders::_1),
        sub_opt);
    while (wait_trans_odom2map && !got_trans_odom2map) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for the Initial Transform between odom and map frame");
      rclcpp::spin_some(shared_from_this());
      usleep(1e6);
    }
    map_2map_transform_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "map2map/transform",
        1,
        std::bind(
            &SGraphsNode::map2map_transform_callback, this, std::placeholders::_1),
        sub_opt);

    odom_sub.subscribe(this, "odom");
    cloud_sub.subscribe(this, "filtered_points");
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(
        ApproxSyncPolicy(32), odom_sub, cloud_sub));
    sync->registerCallback(&SGraphsNode::cloud_callback, this);

    raw_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        1,
        std::bind(&SGraphsNode::raw_odom_callback, this, std::placeholders::_1),
        sub_opt);

    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_points",
        100,
        std::bind(&SGraphsNode::point_cloud_callback, this, std::placeholders::_1),
        sub_opt);

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "gpsimu_driver/imu_data",
        1024,
        std::bind(&SGraphsNode::imu_callback, this, std::placeholders::_1),
        sub_opt);

    room_data_sub = this->create_subscription<s_graphs::msg::RoomsData>(
        "room_segmentation/room_data",
        1,
        std::bind(&SGraphsNode::room_data_callback, this, std::placeholders::_1),
        sub_opt);
    wall_data_sub = this->create_subscription<s_graphs::msg::WallsData>(
        "wall_segmentation/wall_data",
        1,
        std::bind(&SGraphsNode::wall_data_callback, this, std::placeholders::_1),
        sub_opt);
    floor_data_sub = this->create_subscription<s_graphs::msg::RoomData>(
        "floor_plan/floor_data",
        1,
        std::bind(&SGraphsNode::floor_data_callback, this, std::placeholders::_1),
        sub_opt);

    if (this->get_parameter("enable_gps").get_parameter_value().get<bool>()) {
      gps_sub = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
          "gps/geopoint",
          1024,
          std::bind(&SGraphsNode::gps_callback, this, std::placeholders::_1),
          sub_opt);
      nmea_sub = this->create_subscription<nmea_msgs::msg::Sentence>(
          "gpsimu_driver/nmea_sentence",
          1024,
          std::bind(&SGraphsNode::nmea_callback, this, std::placeholders::_1),
          sub_opt);
      navsat_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "gps/navsat",
          1024,
          std::bind(&SGraphsNode::navsat_callback, this, std::placeholders::_1),
          sub_opt);
    }

    callback_group_publisher =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = callback_group_publisher;

    // publishers
    markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "s_graphs/markers", 16, pub_opt);
    odom2map_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "s_graphs/odom2map", 16, pub_opt);
    odom_pose_corrected_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "s_graphs/odom_pose_corrected", 10, pub_opt);
    odom_path_corrected_pub = this->create_publisher<nav_msgs::msg::Path>(
        "s_graphs/odom_path_corrected", 10, pub_opt);

    map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "s_graphs/map_points", 1, pub_opt);
    map_planes_pub = this->create_publisher<s_graphs::msg::PlanesData>(
        "s_graphs/map_planes", 1, pub_opt);
    all_map_planes_pub = this->create_publisher<s_graphs::msg::PlanesData>(
        "s_graphs/all_map_planes", 1, pub_opt);
    read_until_pub = this->create_publisher<std_msgs::msg::Header>(
        "s_graphs/read_until", 32, pub_opt);
    graph_pub = this->create_publisher<reasoning_msgs::msg::Graph>(
        "s_graphs/graph_structure", 32, pub_opt);

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
    load_service_server = this->create_service<s_graphs::srv::LoadGraph>(
        "s_graphs/load",
        std::bind(&SGraphsNode::load_service,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    loop_found = false;
    duplicate_planes_found = false;
    global_optimization = false;
    graph_updated = false;
    prev_edge_count = curr_edge_count = 0;

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

    callback_group_opt_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_keyframe_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_map_pub_timer =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    optimization_timer = this->create_wall_timer(
        std::chrono::seconds(int(graph_update_interval)),
        std::bind(&SGraphsNode::optimization_timer_callback, this),
        callback_group_opt_timer);
    keyframe_timer = this->create_wall_timer(
        std::chrono::seconds(int(keyframe_timer_update_interval)),
        std::bind(&SGraphsNode::keyframe_update_timer_callback, this),
        callback_keyframe_timer);
    map_publish_timer = this->create_wall_timer(
        std::chrono::seconds(int(map_cloud_update_interval)),
        std::bind(&SGraphsNode::map_publish_timer_callback, this),
        callback_map_pub_timer);

    anchor_node = nullptr;
    anchor_edge = nullptr;
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
    this->declare_parameter("use_map2map_transform", false);
    this->declare_parameter("color_r", 0.0);
    this->declare_parameter("color_g", 0.0);
    this->declare_parameter("color_b", 0.0);
    this->declare_parameter("save_timings", false);

    this->declare_parameter("max_keyframes_per_update", 10);
    this->declare_parameter("gps_time_offset", 10);
    this->declare_parameter("gps_edge_stddev_xy", 10000.0);
    this->declare_parameter("gps_edge_stddev_z", 10.0);

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
    this->declare_parameter("keyframe_matching_threshold", 0.1);

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

    this->declare_parameter("optimization_window_size", 10);
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
    covisibility_graph = std::make_shared<GraphSLAM>(
        this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
        this->get_parameter("save_timings").get_parameter_value().get<bool>());
    compressed_graph = std::make_unique<GraphSLAM>(
        this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
        this->get_parameter("save_timings").get_parameter_value().get<bool>());
    visualization_graph = std::make_unique<GraphSLAM>();
    keyframe_updater = std::make_unique<KeyframeUpdater>(shared_from_this());
    plane_analyzer = std::make_unique<PlaneAnalyzer>(shared_from_this());
    loop_mapper = std::make_unique<LoopMapper>(shared_from_this());
    loop_detector = std::make_unique<LoopDetector>(shared_from_this());
    map_cloud_generator = std::make_unique<MapCloudGenerator>();
    inf_calclator = std::make_unique<InformationMatrixCalculator>(shared_from_this());
    nmea_parser = std::make_unique<NmeaSentenceParser>();
    plane_mapper = std::make_unique<PlaneMapper>(shared_from_this());
    inf_room_mapper = std::make_unique<InfiniteRoomMapper>(shared_from_this());
    finite_room_mapper = std::make_unique<FiniteRoomMapper>(shared_from_this());
    floor_mapper = std::make_unique<FloorMapper>();
    graph_visualizer = std::make_unique<GraphVisualizer>(shared_from_this());
    keyframe_mapper = std::make_unique<KeyframeMapper>(shared_from_this());
    gps_mapper = std::make_unique<GPSMapper>(shared_from_this());
    imu_mapper = std::make_unique<IMUMapper>(shared_from_this());
    graph_publisher = std::make_unique<GraphPublisher>();
    wall_mapper = std::make_unique<WallMapper>(shared_from_this());
    room_graph_generator = std::make_unique<RoomGraphGenerator>(shared_from_this());

    main_timer->cancel();
  }

 private:
  /**
   * @brief receive the raw odom msg to publish the corrected odom after s
   *
   */
  void raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    Eigen::Isometry3d odom = odom2isometry(odom_msg);
    geometry_msgs::msg::TransformStamped odom2map_transform;
    Eigen::Isometry3d map2map_trans(trans_map2map.cast<double>());
    Eigen::Matrix4f odom_corrected;
    if (use_map2map_transform) {
      Eigen::Isometry3d odom_trans = map2map_trans * odom;
      odom_corrected = trans_odom2map * odom_trans.matrix().cast<float>();
    } else {
      odom_corrected = trans_odom2map * odom.matrix().cast<float>();
    }
    geometry_msgs::msg::PoseStamped pose_stamped_corrected =
        matrix2PoseStamped(odom_msg->header.stamp, odom_corrected, map_frame_id);
    publish_corrected_odom(pose_stamped_corrected);

    // this is dirty but temp solution for no /clock topic in ros2
    if (use_map2map_transform) {
      Eigen::Matrix4f current_map2map_trans = map2map_trans.matrix().cast<float>();
      odom2map_transform =
          matrix2transform(odom_msg->header.stamp,
                           trans_odom2map * map2map_trans.matrix().cast<float>(),
                           map_frame_id,
                           odom_frame_id);
    } else {
      odom2map_transform = matrix2transform(
          odom_msg->header.stamp, trans_odom2map, map_frame_id, odom_frame_id);
    }
    odom2map_broadcaster->sendTransform(odom2map_transform);
  }

  /**
   * @brief receive the raw pointcloud
   *
   * @param cloud_msg
   */
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    cloud_queue_mutex.lock();
    cloud_queue.push_back(cloud_msg);
    cloud_queue_mutex.unlock();
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

  /**
   * @brief receive the between loaded and new map frame
   * @param map2odom_pose_msg
   */
  void map2map_transform_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
    std::cout << "inside callback" << std::endl;
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose_msg->pose.orientation.w,
                                              pose_msg->pose.orientation.x,
                                              pose_msg->pose.orientation.y,
                                              pose_msg->pose.orientation.z)
                               .toRotationMatrix();
    trans_map2map.setIdentity();
    trans_map2map.block<3, 3>(0, 0) = mat3;
    trans_map2map(0, 3) = pose_msg->pose.position.x;
    trans_map2map(1, 3) = pose_msg->pose.position.y;
    trans_map2map(2, 3) = pose_msg->pose.position.z;
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
      floor_mapper->lookup_floors(covisibility_graph,
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
        if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 2) {
          float x_width = PlaneUtils::width_between_planes(room_data.x_planes[0],
                                                           room_data.x_planes[1]);
          float y_width = PlaneUtils::width_between_planes(room_data.y_planes[0],
                                                           room_data.y_planes[1]);

          if (x_width < 0.5 || y_width < 0.5) continue;

          int current_room_id;
          bool duplicate_planes_rooms =
              finite_room_mapper->lookup_rooms(covisibility_graph,
                                               room_data,
                                               x_vert_planes,
                                               y_vert_planes,
                                               dupl_x_vert_planes,
                                               dupl_y_vert_planes,
                                               x_infinite_rooms,
                                               y_infinite_rooms,
                                               rooms_vec,
                                               current_room_id);
          // generate local graph per room
          extract_keyframes_from_room(rooms_vec[current_room_id]);
          graph_mutex.lock();
          room_local_graph_id_queue.push_back(current_room_id);
          graph_mutex.unlock();
          if (duplicate_planes_rooms) duplicate_planes_found = true;
        }
        // x infinite_room
        else if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 0) {
          float x_width = PlaneUtils::width_between_planes(room_data.x_planes[0],
                                                           room_data.x_planes[1]);
          if (x_width < 0.5) continue;

          bool duplicate_planes_x_inf_rooms = inf_room_mapper->lookup_infinite_rooms(
              covisibility_graph,
              PlaneUtils::plane_class::X_VERT_PLANE,
              room_data,
              x_vert_planes,
              y_vert_planes,
              dupl_x_vert_planes,
              dupl_y_vert_planes,
              x_infinite_rooms,
              y_infinite_rooms,
              rooms_vec);

          if (duplicate_planes_x_inf_rooms) duplicate_planes_found = true;
        }
        // y infinite_room
        else if (room_data.x_planes.size() == 0 && room_data.y_planes.size() == 2) {
          float y_width = PlaneUtils::width_between_planes(room_data.y_planes[0],
                                                           room_data.y_planes[1]);
          if (y_width < 0.5) continue;

          bool duplicate_planes_y_inf_rooms = inf_room_mapper->lookup_infinite_rooms(
              covisibility_graph,
              PlaneUtils::plane_class::Y_VERT_PLANE,
              room_data,
              x_vert_planes,
              y_vert_planes,
              dupl_x_vert_planes,
              dupl_y_vert_planes,
              x_infinite_rooms,
              y_infinite_rooms,
              rooms_vec);

          if (duplicate_planes_y_inf_rooms) duplicate_planes_found = true;
        }
      }

      room_data_queue_mutex.lock();
      room_data_queue.pop_front();
      room_data_queue_mutex.unlock();
    }
  }

  /**
   *@brief extract all the keyframes from the found room
   **/

  void extract_keyframes_from_room(Rooms& current_room) {
    // check if the current robot pose lies in a room
    if (rooms_vec.empty()) return;

    // if current room is not empty then get the keyframes in the room
    if (current_room.node != nullptr) {
      Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
      std::map<int, s_graphs::KeyFrame::Ptr> room_keyframes =
          room_graph_generator->get_keyframes_inside_room(
              current_room, x_vert_planes, y_vert_planes, keyframes);
      // create the local graph for that room
      room_graph_generator->generate_local_graph(
          keyframe_mapper, covisibility_graph, room_keyframes, odom2map, current_room);
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
    if (use_map2map_transform) {
      Eigen::Isometry3d map2map_trans(trans_map2map.cast<double>());
      Eigen::Quaterniond quaternion(map2map_trans.rotation());
      Eigen::Isometry3d odom_trans = map2map_trans * odom;
      Eigen::Quaterniond odom_quaternion(odom_trans.rotation());

      KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom_trans, accum_d, cloud));
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      keyframe_queue.push_back(keyframe);
    } else {
      KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      keyframe_queue.push_back(keyframe);
    }
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
   * (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    if (keyframe_queue.empty()) {
      // std::cout << "keyframe_queue is empty " << std::endl;
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    graph_mutex.lock();
    int num_processed = keyframe_mapper->map_keyframes(covisibility_graph,
                                                       odom2map,
                                                       keyframe_queue,
                                                       keyframes,
                                                       new_keyframes,
                                                       anchor_node,
                                                       anchor_edge,
                                                       keyframe_hash);
    graph_mutex.unlock();

    // perform planar segmentation
    for (int i = 0; i < new_keyframes.size(); i++) {
      // perform planar segmentation
      if (extract_planar_surfaces) {
        std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec =
            plane_analyzer->extract_segmented_planes(new_keyframes[i]->cloud);
        graph_mutex.lock();
        plane_mapper->map_extracted_planes(covisibility_graph,
                                           new_keyframes[i],
                                           extracted_cloud_vec,
                                           x_vert_planes,
                                           y_vert_planes,
                                           hort_planes);
        graph_mutex.unlock();
      }
    }

    std_msgs::msg::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + rclcpp::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub->publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub->publish(read_until);

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.erase(keyframe_queue.begin(),
                         keyframe_queue.begin() + num_processed + 1);

    return true;
  }

  void wall_data_callback(const s_graphs::msg::WallsData::SharedPtr walls_msg) {
    for (int j = 0; j < walls_msg->walls.size(); j++) {
      std::vector<s_graphs::msg::PlaneData> x_planes_msg = walls_msg->walls[j].x_planes;
      std::vector<s_graphs::msg::PlaneData> y_planes_msg = walls_msg->walls[j].y_planes;

      if (x_planes_msg.size() != 2 && y_planes_msg.size() != 2) continue;

      Eigen::Vector3d wall_pose;
      wall_pose << walls_msg->walls[j].wall_center.position.x,
          walls_msg->walls[j].wall_center.position.y,
          walls_msg->walls[j].wall_center.position.z;

      Eigen::Vector3d wall_point;
      wall_point << walls_msg->walls[j].wall_point.x, walls_msg->walls[j].wall_point.y,
          walls_msg->walls[j].wall_point.z;

      wall_mapper->factor_wall(covisibility_graph,
                               wall_pose,
                               wall_point,
                               x_planes_msg,
                               y_planes_msg,
                               x_vert_planes,
                               y_vert_planes);
    }
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

    return gps_mapper->map_gps_data(covisibility_graph, gps_queue, keyframes);
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

    return imu_mapper->map_imu_data(
        covisibility_graph, tf_buffer, imu_queue, keyframes, base_frame_id);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then
   * optimizes the pose graph
   * @param event
   */
  void keyframe_update_timer_callback() {
    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if (!keyframe_updated) {
      std_msgs::msg::Header read_until;
      read_until.stamp = this->now() + rclcpp::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub->publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub->publish(read_until);
    }

    if (!keyframe_updated & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // publish mapped planes
    publish_mapped_planes(x_vert_planes, y_vert_planes);

    // flush the room poses from room detector and no need to return if no rooms found
    flush_room_data_queue();

    // flush the floor poses from the floor planner and no need to return if no floors
    // found
    flush_floor_data_queue();

    // loop detection
    std::vector<Loop::Ptr> loops =
        loop_detector->detect(keyframes, new_keyframes, *covisibility_graph);
    if (loops.size() > 0) {
      loop_found = true;
      loop_mapper->add_loops(covisibility_graph, loops, graph_mutex);
    }

    graph_mutex.lock();
    std::transform(new_keyframes.begin(),
                   new_keyframes.end(),
                   std::inserter(keyframes, keyframes.end()),
                   [](const KeyFrame::Ptr& k) { return std::make_pair(k->id(), k); });

    new_keyframes.clear();
    graph_mutex.unlock();

    // move the first node anchor position to the current estimate of the first node
    // pose so the first node moves freely while trying to stay around the origin
    if (anchor_node && this->get_parameter("fix_first_node_adaptive")
                           .get_parameter_value()
                           .get<bool>()) {
      Eigen::Isometry3d anchor_target =
          static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(),
                   keyframes.end(),
                   snapshot.begin(),
                   [=](const std::pair<int, KeyFrame::Ptr>& k) {
                     return std::make_shared<KeyFrameSnapshot>(k.second);
                   });
    keyframes_snapshot.swap(snapshot);

    for (const auto& keyframe_snapshot : keyframes_snapshot) {
      if (!keyframe_snapshot->k_marginalized)
        keyframes_snapshot_queue.push(keyframe_snapshot);
    }

    for (const auto& keyframe : keyframes) {
      complete_keyframes_queue.push(keyframe.second);
    }

    std::vector<VerticalPlanes> current_x_planes(x_vert_planes.size());
    std::transform(x_vert_planes.begin(),
                   x_vert_planes.end(),
                   current_x_planes.begin(),
                   [](const std::pair<int, VerticalPlanes>& x_plane) {
                     return VerticalPlanes(x_plane.second, true);
                   });
    x_planes_snapshot.swap(current_x_planes);

    std::vector<VerticalPlanes> current_y_planes(y_vert_planes.size());
    std::transform(y_vert_planes.begin(),
                   y_vert_planes.end(),
                   current_y_planes.begin(),
                   [](const std::pair<int, VerticalPlanes>& y_plane) {
                     return VerticalPlanes(y_plane.second, true);
                   });
    y_planes_snapshot.swap(current_y_planes);

    std::vector<HorizontalPlanes> current_hort_planes(hort_planes.size());
    std::transform(hort_planes.begin(),
                   hort_planes.end(),
                   current_hort_planes.begin(),
                   [](const std::pair<int, HorizontalPlanes>& hort_plane) {
                     return HorizontalPlanes(hort_plane.second, true);
                   });
    hort_planes_snapshot.swap(current_hort_planes);

    std::vector<InfiniteRooms> curent_x_inf_rooms(x_infinite_rooms.size());
    std::transform(x_infinite_rooms.begin(),
                   x_infinite_rooms.end(),
                   curent_x_inf_rooms.begin(),
                   [](const std::pair<int, InfiniteRooms>& x_inf_room) {
                     return InfiniteRooms(x_inf_room.second, true);
                   });
    x_inf_rooms_snapshot.swap(curent_x_inf_rooms);

    std::vector<InfiniteRooms> curent_y_inf_rooms(y_infinite_rooms.size());
    std::transform(y_infinite_rooms.begin(),
                   y_infinite_rooms.end(),
                   curent_y_inf_rooms.begin(),
                   [](const std::pair<int, InfiniteRooms>& y_inf_room) {
                     return InfiniteRooms(y_inf_room.second, true);
                   });
    y_inf_rooms_snapshot.swap(curent_y_inf_rooms);

    std::vector<Rooms> curent_rooms(rooms_vec.size());
    std::transform(
        rooms_vec.begin(),
        rooms_vec.end(),
        curent_rooms.begin(),
        [](const std::pair<int, Rooms>& room) { return Rooms(room.second, true); });
    rooms_vec_snapshot.swap(curent_rooms);

    std::vector<Floors> current_floors(floors_vec.size());
    std::transform(floors_vec.begin(),
                   floors_vec.end(),
                   current_floors.begin(),
                   [](const std::pair<int, Floors>& floor) { return floor.second; });
    floors_vec_snapshot.swap(current_floors);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then
   * optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback() {
    if (keyframes.empty()) return;

    int num_iterations = this->get_parameter("g2o_solver_num_iterations")
                             .get_parameter_value()
                             .get<int>();

    curr_edge_count = covisibility_graph->retrieve_total_nbr_of_edges();
    if (curr_edge_count <= prev_edge_count) {
      return;
    }

    graph_mutex.lock();
    const int keyframe_id = keyframes.rbegin()->first;
    graph_mutex.unlock();

    graph_mutex.lock();
    if (!loop_found && !duplicate_planes_found) {
      GraphUtils::copy_windowed_graph(
          optimization_window_size, covisibility_graph, compressed_graph, keyframes);
      global_optimization = false;
    } else if (loop_found && !duplicate_planes_found) {
      GraphUtils::copy_graph(covisibility_graph, compressed_graph, keyframes);
      loop_found = false;
      global_optimization = true;
    } else if (!loop_found && duplicate_planes_found) {
      GraphUtils::copy_graph(covisibility_graph, compressed_graph, keyframes);
      duplicate_planes_found = false;
      global_optimization = true;
    } else if (loop_found && duplicate_planes_found) {
      GraphUtils::copy_graph(covisibility_graph, compressed_graph, keyframes);
      duplicate_planes_found = false;
      loop_found = false;
      global_optimization = true;
    }
    graph_mutex.unlock();

    // optimize the pose graph
    try {
      if (!global_optimization)
        compressed_graph->optimize("local", num_iterations);
      else {
        compressed_graph->optimize("global", num_iterations);
      }
    } catch (std::invalid_argument& e) {
      std::cout << e.what() << std::endl;
      throw 1;
    }

    graph_mutex.lock();
    GraphUtils::update_graph(compressed_graph,
                             keyframes,
                             x_vert_planes,
                             y_vert_planes,
                             rooms_vec,
                             x_infinite_rooms,
                             y_infinite_rooms,
                             floors_vec);

    Eigen::Isometry3d trans = keyframes[keyframe_id]->node->estimate() *
                              keyframes[keyframe_id]->odom.inverse();

    // publish tf
    geometry_msgs::msg::TransformStamped ts =
        matrix2transform(keyframes[keyframe_id]->stamp,
                         trans.matrix().cast<float>(),
                         map_frame_id,
                         odom_frame_id);
    odom2map_pub->publish(ts);
    graph_mutex.unlock();

    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    int counter = 0;
    for (const auto& room_local_graph_id : room_local_graph_id_queue) {
      broadcast_room_graph(room_local_graph_id, num_iterations);
      counter++;
    }

    if (!room_local_graph_id_queue.empty()) {
      graph_mutex.lock();
      room_local_graph_id_queue.erase(room_local_graph_id_queue.begin(),
                                      room_local_graph_id_queue.begin() + counter);
      graph_mutex.unlock();
    }

    graph_updated = true;
    prev_edge_count = curr_edge_count;
  }

  /**
   * @brief optimize the local room graphs
   *
   * @param room_id
   * @param num_iterations
   */
  void broadcast_room_graph(const int room_id, const int num_iterations) {
    graph_mutex.lock();
    // optimize_room_local_graph
    rooms_vec[room_id].local_graph->optimize("room-local", num_iterations);
    GraphUtils::set_marginalize_info(rooms_vec[room_id].local_graph,
                                     covisibility_graph,
                                     rooms_vec[room_id].room_keyframes);
    graph_mutex.unlock();
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_publish_timer_callback() {
    if (map_points_pub->get_subscription_count() < 0 || !graph_updated) {
      return;
    }

    int current_loop = 0;
    KeyFrameSnapshot::Ptr current_snapshot;
    while (keyframes_snapshot_queue.pop(current_snapshot)) {
      if (current_loop == 0) current_keyframes_snapshot.clear();
      current_keyframes_snapshot.push_back(current_snapshot);
      current_loop++;
    }

    current_loop = 0;
    KeyFrame::Ptr current_keyframe;
    while (complete_keyframes_queue.pop(current_keyframe)) {
      if (current_loop == 0) current_keyframes.clear();
      current_keyframes.push_back(current_keyframe);
      current_loop++;
    }

    auto cloud =
        map_cloud_generator->generate(current_keyframes_snapshot, map_cloud_resolution);
    if (!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = current_keyframes_snapshot.back()->cloud->header.stamp;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(
        new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    auto current_time = this->now();
    auto markers = graph_visualizer->create_marker_array(
        current_time,
        covisibility_graph->graph.get(),
        x_planes_snapshot,
        y_planes_snapshot,
        hort_planes_snapshot,
        x_inf_rooms_snapshot,
        y_inf_rooms_snapshot,
        rooms_vec_snapshot,
        loop_detector->get_distance_thresh() * 2.0,
        current_keyframes,
        floors_vec_snapshot);

    graph_visualizer->create_compressed_graph(current_time,
                                              global_optimization,
                                              false,
                                              compressed_graph->graph.get(),
                                              x_planes_snapshot,
                                              y_planes_snapshot,
                                              hort_planes_snapshot);

    markers_pub->publish(markers);
    publish_all_mapped_planes(x_planes_snapshot, y_planes_snapshot);
    map_points_pub->publish(*cloud_msg);
    publish_graph();
  }

  /**
   * @brief generate graph structure and publish it
   * @param event
   */
  void publish_graph() {
    std::string graph_type;
    if (std::string("/robot1") == this->get_namespace()) {
      graph_type = "Prior";
    } else {
      graph_type = "Online";
    }
    auto graph_structure =
        graph_publisher->publish_graph(covisibility_graph->graph.get(),
                                       "Online",
                                       x_vert_planes_prior,
                                       y_vert_planes_prior,
                                       rooms_vec_prior,
                                       x_planes_snapshot,
                                       y_planes_snapshot,
                                       rooms_vec_snapshot,
                                       x_inf_rooms_snapshot,
                                       y_inf_rooms_snapshot);
    graph_structure.name = graph_type;
    graph_pub->publish(graph_structure);
  }

  /**
   * @brief publish the mapped plane information from the last n keyframes
   *
   */
  void publish_mapped_planes(
      std::unordered_map<int, VerticalPlanes> x_vert_planes_snapshot,
      std::unordered_map<int, VerticalPlanes> y_vert_planes_snapshot) {
    if (keyframes.empty()) return;

    std::map<int, KeyFrame::Ptr> keyframe_window;
    auto it = keyframes.rbegin();
    for (; it != keyframes.rend() && keyframe_window.size() < keyframe_window_size;
         ++it) {
      keyframe_window.insert(*it);
    }

    std::map<int, int> unique_x_plane_ids, unique_y_plane_ids;
    for (std::map<int, KeyFrame::Ptr>::reverse_iterator it = keyframe_window.rbegin();
         it != keyframe_window.rend();
         ++it) {
      for (const auto& x_plane_id : (it)->second->x_plane_ids) {
        auto result = unique_x_plane_ids.insert(std::pair<int, int>(x_plane_id, 1));
      }

      for (const auto& y_plane_id : (it)->second->y_plane_ids) {
        auto result = unique_y_plane_ids.insert(std::pair<int, int>(y_plane_id, 1));
      }
    }

    s_graphs::msg::PlanesData vert_planes_data;
    vert_planes_data.header.stamp = keyframes.rbegin()->second->stamp;
    for (const auto& unique_x_plane_id : unique_x_plane_ids) {
      auto local_x_vert_plane = x_vert_planes_snapshot.find(unique_x_plane_id.first);

      if (local_x_vert_plane == x_vert_planes_snapshot.end()) continue;
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs =
          (local_x_vert_plane->second).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (local_x_vert_plane->second).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data :
           (local_x_vert_plane->second).cloud_seg_map->points) {
        geometry_msgs::msg::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.x_planes.push_back(plane_data);
    }

    for (const auto& unique_y_plane_id : unique_y_plane_ids) {
      auto local_y_vert_plane = y_vert_planes_snapshot.find(unique_y_plane_id.first);

      if (local_y_vert_plane == y_vert_planes_snapshot.end()) continue;
      s_graphs::msg::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs =
          (local_y_vert_plane->second).plane_node->estimate().coeffs();
      // correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
      // mapped_plane_coeffs);
      plane_data.id = (local_y_vert_plane->second).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for (const auto& plane_point_data :
           (local_y_vert_plane->second).cloud_seg_map->points) {
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
    vert_planes_data.header.stamp = keyframes.rbegin()->second->stamp;
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
      if (x_vert_plane.type == "Prior") {
        plane_data.data_source = "PRIOR";
      } else {
        plane_data.data_source = "Online";
      }
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
      if (y_vert_plane.type == "Prior") {
        plane_data.data_source = "PRIOR";
      } else {
        plane_data.data_source = "Online";
      }
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

    covisibility_graph->save(directory + "/graph.g2o");
    for (int i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }
    for (int i = 0; i < x_vert_planes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;
      x_vert_planes[i].save(sst.str(), 'x');
    }
    for (int i = 0; i < y_vert_planes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;
      y_vert_planes[i].save(sst.str(), 'y');
    }
    for (int i = 0; i < rooms_vec.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;
      rooms_vec[i].save(sst.str());
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

    snapshot = keyframes_snapshot;

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

  bool load_service(const std::shared_ptr<s_graphs::srv::LoadGraph::Request> req,
                    std::shared_ptr<s_graphs::srv::LoadGraph::Response> res) {
    std::string directory = req->destination;
    std::vector<std::string> keyframe_directories, y_planes_directories,
        x_planes_directories, room_directories;
    ;
    std::stringstream sst;
    bool keyframe_load_success = false;
    bool plane_load_success = false;
    bool room_load_success = false;
    rclcpp::Time stamp;
    Eigen::Isometry3d odom;
    double accum_distance;
    pcl::PointCloud<PointT>::ConstPtr cloud;
    sst << boost::format("%s") % directory;
    std::map<int, KeyFrame::Ptr> loaded_keyframes;
    g2o::SparseOptimizer* local_graph;
    local_graph = covisibility_graph->graph.get();
    VerticalPlanes load_x_planes;
    // load_x_planes.load(sst.str(), local_graph);
    boost::filesystem::path parentPath(directory);
    int numSubdirectories = 0;
    int iterator = 0;
    if (boost::filesystem::is_directory(parentPath)) {
      for (const auto& entry : boost::filesystem::directory_iterator(parentPath)) {
        if (boost::filesystem::is_directory(entry.path())) {
          keyframe_directories.push_back(entry.path().string());
        }
      }
    }
    std::sort(keyframe_directories.begin(), keyframe_directories.end());

    for (int i = 0; i < keyframe_directories.size(); i++) {
      auto keyframe = std::make_shared<KeyFrame>(stamp, odom, accum_distance, cloud);
      keyframe->node = covisibility_graph->add_se3_node(keyframe->odom);
      keyframe_load_success = keyframe->load(keyframe_directories[i], local_graph);
      loaded_keyframes.insert({keyframe->id(), keyframe});
    }

    for (int i = 0; i < loaded_keyframes.size() - 1; i++) {
      KeyFrame::Ptr& prev_keyframe = loaded_keyframes[i];
      KeyFrame::Ptr& next_keyframe = loaded_keyframes[i + 1];

      Eigen::Isometry3d relative_pose =
          next_keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
          next_keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = covisibility_graph->add_se3_edge(
          next_keyframe->node, prev_keyframe->node, relative_pose, information);
      covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    }
    keyframes = loaded_keyframes;
    std::cout << " loaded keyframes size : " << keyframes.size() << std::endl;

    for (const auto& directoryPath : keyframe_directories) {
      for (const auto& entry : boost::filesystem::directory_iterator(directoryPath)) {
        if (boost::filesystem::is_directory(entry)) {
          size_t lastSlashPos = entry.path().string().find_last_of("/");

          // Extract the substring after the last '/'
          std::string lastPart = entry.path().string().substr(lastSlashPos + 1);

          if (lastPart == "y_planes") {
            y_planes_directories.push_back(entry.path().string());
          } else if (lastPart == "x_planes") {
            x_planes_directories.push_back(entry.path().string());
          }
        }
      }
    }

    for (int i = 0; i < y_planes_directories.size(); i++) {
      VerticalPlanes vert_plane;
      g2o::Plane3D loaded_plane;
      g2o::VertexPlane* p_node =
          covisibility_graph->add_plane_node(loaded_plane.coeffs());
      vert_plane.plane_node = p_node;
      plane_load_success = vert_plane.load(y_planes_directories[i], local_graph, "y");
      y_vert_planes.insert({vert_plane.id, vert_plane});
    }

    for (int i = 0; i < x_planes_directories.size(); i++) {
      VerticalPlanes vert_plane;
      g2o::Plane3D loaded_plane;
      g2o::VertexPlane* p_node =
          covisibility_graph->add_plane_node(loaded_plane.coeffs());
      vert_plane.plane_node = p_node;
      plane_load_success = vert_plane.load(x_planes_directories[i], local_graph, "x");
      x_vert_planes.insert({vert_plane.id, vert_plane});
    }
    for (auto& y_vert_plane : y_vert_planes) {
      Eigen::Matrix3d plane_information_mat =
          Eigen::Matrix3d::Identity() * plane_information;
      plane_information_mat(3, 3) = plane_information_mat(3, 3) / 10;

      assert(y_vert_plane.second.cloud_seg_body_vec.size() ==
             y_vert_plane.second.keyframe_node_vec.size());

      for (int j = 0; j < y_vert_plane.second.keyframe_node_vec.size(); j++) {
        // std::cout << "Y keyframe node id : "
        //           << y_vert_planes[i].keyframe_node_vec[j]->id() << std::endl;
        // std::cout << "plane : " << i << "  cloud : " << j << std::endl;
        g2o::Plane3D det_plane_body_frame = Eigen::Vector4d(
            y_vert_plane.second.cloud_seg_body_vec[j]->back().normal_x,
            y_vert_plane.second.cloud_seg_body_vec[j]->back().normal_y,
            y_vert_plane.second.cloud_seg_body_vec[j]->back().normal_z,
            y_vert_plane.second.cloud_seg_body_vec[j]->back().curvature);

        auto edge = covisibility_graph->add_se3_plane_edge(
            y_vert_plane.second.keyframe_node_vec[j],
            y_vert_plane.second.plane_node,
            det_plane_body_frame.coeffs(),
            plane_information_mat);
        covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
      }
    }
    for (auto& x_vert_plane : x_vert_planes) {
      Eigen::Matrix3d plane_information_mat =
          Eigen::Matrix3d::Identity() * plane_information;
      plane_information_mat(3, 3) = plane_information_mat(3, 3) / 10;

      assert(x_vert_plane.second.cloud_seg_body_vec.size() ==
             x_vert_plane.second.keyframe_node_vec.size());
      for (int j = 0; j < x_vert_plane.second.keyframe_node_vec.size(); j++) {
        g2o::Plane3D det_plane_body_frame = Eigen::Vector4d(
            x_vert_plane.second.cloud_seg_body_vec[j]->back().normal_x,
            x_vert_plane.second.cloud_seg_body_vec[j]->back().normal_y,
            x_vert_plane.second.cloud_seg_body_vec[j]->back().normal_z,
            x_vert_plane.second.cloud_seg_body_vec[j]->back().curvature);

        auto edge = covisibility_graph->add_se3_plane_edge(
            x_vert_plane.second.keyframe_node_vec[j],
            x_vert_plane.second.plane_node,
            det_plane_body_frame.coeffs(),
            plane_information_mat);
        covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
      }
    }
    for (const auto& entry : boost::filesystem::directory_iterator(parentPath)) {
      if (boost::filesystem::is_directory(entry)) {
        boost::filesystem::path room_data_path = entry.path();
        room_data_path /= "room_data";
        if (boost::filesystem::exists(room_data_path)) {
          room_directories.push_back(entry.path().string());
        }
      }
    }
    std::sort(room_directories.begin(), room_directories.end());
    for (int i = 0; i < room_directories.size(); i++) {
      Rooms loaded_room;
      Eigen::Isometry3d room_center;
      auto r_node = covisibility_graph->add_room_node(room_center);
      loaded_room.node = r_node;
      room_load_success =
          loaded_room.load(room_directories[i], covisibility_graph->graph.get());
      rooms_vec.insert({loaded_room.id, loaded_room});
    }
    Eigen::Matrix<double, 2, 2> information_room_planes;
    information_room_planes.setZero();
    information_room_planes(0, 0) = room_information;
    information_room_planes(1, 1) = room_information;
    for (int i = 0; i < rooms_vec.size(); i++) {
      auto edge_room_planes =
          covisibility_graph->add_room_4planes_edge(rooms_vec[i].node,
                                                    rooms_vec[i].plane_x1_node,
                                                    rooms_vec[i].plane_x2_node,
                                                    rooms_vec[i].plane_y1_node,
                                                    rooms_vec[i].plane_y2_node,
                                                    information_room_planes);
      covisibility_graph->add_robust_kernel(edge_room_planes, "Huber", 1.0);
    }
    res->success = true;
    return true;
  }

 private:
  // ROS
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr keyframe_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::CallbackGroup::SharedPtr callback_group_publisher;

  rclcpp::CallbackGroup::SharedPtr callback_group_opt_timer;
  rclcpp::CallbackGroup::SharedPtr callback_keyframe_timer;
  rclcpp::CallbackGroup::SharedPtr callback_map_pub_timer;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                          sensor_msgs::msg::PointCloud2>
      ApproxSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_sub;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<s_graphs::msg::RoomsData>::SharedPtr room_data_sub;
  rclcpp::Subscription<s_graphs::msg::WallsData>::SharedPtr wall_data_sub;
  rclcpp::Subscription<s_graphs::msg::RoomData>::SharedPtr floor_data_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_odom2map_sub,
      map_2map_transform_sub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map, trans_map2map;
  bool wait_trans_odom2map, got_trans_odom2map, use_map2map_transform;
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
  rclcpp::Publisher<s_graphs::msg::PlanesData>::SharedPtr map_planes_pub;
  rclcpp::Publisher<s_graphs::msg::PlanesData>::SharedPtr all_map_planes_pub;
  rclcpp::Publisher<reasoning_msgs::msg::Graph>::SharedPtr graph_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom2map_broadcaster;

  rclcpp::Service<s_graphs::srv::DumpGraph>::SharedPtr dump_service_server;
  rclcpp::Service<s_graphs::srv::LoadGraph>::SharedPtr load_service_server;
  rclcpp::Service<s_graphs::srv::SaveMap>::SharedPtr save_map_service_server;

  // odom queue
  std::mutex odom_queue_mutex;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue;

  // cloud queue
  std::mutex cloud_queue_mutex;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_queue;

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

  // vertical and horizontal planes
  std::deque<int> room_local_graph_id_queue;
  int optimization_window_size;
  bool loop_found, duplicate_planes_found;
  bool global_optimization;
  int keyframe_window_size;
  bool extract_planar_surfaces;
  bool constant_covariance;
  double min_plane_points;
  double infinite_room_information;
  double room_information, plane_information;
  std::unordered_map<int, VerticalPlanes> x_vert_planes,
      y_vert_planes;  // vertically segmented planes
  std::vector<VerticalPlanes> x_vert_planes_prior, y_vert_planes_prior;
  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      dupl_y_vert_planes;  // vertically segmented planes
  std::unordered_map<int, HorizontalPlanes>
      hort_planes;  // horizontally segmented planes
  std::unordered_map<int, InfiniteRooms> x_infinite_rooms,
      y_infinite_rooms;                      // infinite_rooms segmented from planes
  std::unordered_map<int, Rooms> rooms_vec;  // rooms segmented from planes
  std::vector<Rooms> rooms_vec_prior;
  std::unordered_map<int, Floors> floors_vec;
  int prev_edge_count, curr_edge_count;

  std::vector<VerticalPlanes> x_planes_snapshot, y_planes_snapshot;
  std::vector<HorizontalPlanes> hort_planes_snapshot;
  std::vector<Rooms> rooms_vec_snapshot;
  std::vector<Floors> floors_vec_snapshot;
  std::vector<InfiniteRooms> x_inf_rooms_snapshot, y_inf_rooms_snapshot;

  // room data queue
  std::mutex room_data_queue_mutex, floor_data_mutex;
  std::deque<s_graphs::msg::RoomsData> room_data_queue;
  std::deque<s_graphs::msg::RoomData> floor_data_queue;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  boost::lockfree::spsc_queue<KeyFrameSnapshot::Ptr> keyframes_snapshot_queue{1000};
  boost::lockfree::spsc_queue<KeyFrame::Ptr> complete_keyframes_queue{1000};
  std::vector<KeyFrame::Ptr> current_keyframes;
  std::vector<KeyFrameSnapshot::Ptr> current_keyframes_snapshot;

  std::mutex graph_mutex;
  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  std::map<int, KeyFrame::Ptr> keyframes;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::shared_ptr<GraphSLAM> covisibility_graph;
  std::unique_ptr<GraphSLAM> compressed_graph;
  std::unique_ptr<GraphSLAM> visualization_graph;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<LoopMapper> loop_mapper;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<PlaneAnalyzer> plane_analyzer;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<WallMapper> wall_mapper;
  std::unique_ptr<PlaneMapper> plane_mapper;
  std::unique_ptr<InfiniteRoomMapper> inf_room_mapper;
  std::unique_ptr<FiniteRoomMapper> finite_room_mapper;
  std::unique_ptr<FloorMapper> floor_mapper;
  std::unique_ptr<GraphVisualizer> graph_visualizer;
  std::unique_ptr<KeyframeMapper> keyframe_mapper;
  std::unique_ptr<GPSMapper> gps_mapper;
  std::unique_ptr<IMUMapper> imu_mapper;
  std::unique_ptr<GraphPublisher> graph_publisher;
  std::unique_ptr<RoomGraphGenerator> room_graph_generator;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor multi_executor;
  auto node = std::make_shared<s_graphs::SGraphsNode>();
  multi_executor.add_node(node);
  multi_executor.spin();
  rclcpp::shutdown();
  return 0;
}
