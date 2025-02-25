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

#include "s_graphs/common/s_graphs.hpp"

namespace s_graphs {

SGraphsNode::SGraphsNode() : Node("s_graphs_node") {
  this->set_dump_directory();
  anchor_node = nullptr;
  anchor_edge = nullptr;
  // one time timer to initialize the classes with the current node obj
  main_timer = this->create_wall_timer(std::chrono::seconds(1),
                                       std::bind(&SGraphsNode::init_subclass, this));
  // init ros parameters
  this->declare_ros_params();
  base_frame_id = "";
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
  map_cloud_pub_resolution = this->get_parameter("map_cloud_pub_resolution")
                                 .get_parameter_value()
                                 .get<double>();

  fast_mapping = this->get_parameter("fast_mapping").get_parameter_value().get<bool>();
  save_dense_map =
      this->get_parameter("save_dense_map").get_parameter_value().get<bool>();
  viz_dense_map =
      this->get_parameter("viz_dense_map").get_parameter_value().get<bool>();
  viz_all_floor_cloud =
      this->get_parameter("viz_all_floor_cloud").get_parameter_value().get<bool>();
  wait_trans_odom2map =
      this->get_parameter("wait_trans_odom2map").get_parameter_value().get<bool>();
  got_trans_odom2map = false;
  trans_odom2map.setIdentity();
  odom_path_vec.clear();

  max_keyframes_per_update =
      this->get_parameter("max_keyframes_per_update").get_parameter_value().get<int>();

  gps_time_offset =
      this->get_parameter("gps_time_offset").get_parameter_value().get<int>();
  imu_time_offset =
      this->get_parameter("imu_time_offset").get_parameter_value().get<int>();
  enable_imu_orientation =
      this->get_parameter("enable_imu_orientation").get_parameter_value().get<bool>();
  enable_imu_acceleration =
      this->get_parameter("enable_imu_acceleration").get_parameter_value().get<bool>();
  imu_orientation_edge_stddev = this->get_parameter("imu_orientation_edge_stddev")
                                    .get_parameter_value()
                                    .get<double>();
  imu_acceleration_edge_stddev = this->get_parameter("imu_acceleration_edge_stddev")
                                     .get_parameter_value()
                                     .get<double>();

  optimization_window_size =
      this->get_parameter("optimization_window_size").get_parameter_value().get<int>();

  keyframe_window_size =
      this->get_parameter("keyframe_window_size").get_parameter_value().get<int>();
  extract_planar_surfaces =
      this->get_parameter("extract_planar_surfaces").get_parameter_value().get<bool>();
  constant_covariance =
      this->get_parameter("constant_covariance").get_parameter_value().get<bool>();

  infinite_room_information = this->get_parameter("infinite_room_information")
                                  .get_parameter_value()
                                  .get<double>();
  room_information =
      this->get_parameter("room_information").get_parameter_value().get<double>();

  int odom_pc_sync_queue =
      this->get_parameter("odom_pc_sync_queue").get_parameter_value().get<int>();

  floor_level_viz_height =
      this->get_parameter("floor_level_viz_height").get_parameter_value().get<double>();
  keyframe_viz_height =
      this->get_parameter("keyframe_viz_height").get_parameter_value().get<double>();

  wall_viz_height =
      this->get_parameter("wall_viz_height").get_parameter_value().get<double>();

  room_viz_height =
      this->get_parameter("room_viz_height").get_parameter_value().get<double>();

  floor_node_viz_height =
      this->get_parameter("floor_node_viz_height").get_parameter_value().get<double>();

  use_floor_color_for_map =
      this->get_parameter("use_floor_color_for_map").get_parameter_value().get<bool>();

  always_publish_map =
      this->get_parameter("always_publish_map").get_parameter_value().get<bool>();

  // tfs
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  odom2map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  floor_map_static_transforms =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  callback_group_subscriber =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_subscriber;

  // subscribers
  init_odom2map_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "odom2map/initial_pose",
      1,
      std::bind(&SGraphsNode::init_map2odom_pose_callback, this, std::placeholders::_1),
      sub_opt);
  while (wait_trans_odom2map && !got_trans_odom2map) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for the Initial Transform between odom and map frame");
    rclcpp::spin_some(shared_from_this());
    usleep(1e6);
  }

  odom_sub.subscribe(this, "odom");
  cloud_sub.subscribe(this, "filtered_points");
  sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(
      ApproxSyncPolicy(odom_pc_sync_queue), odom_sub, cloud_sub));
  sync->registerCallback(&SGraphsNode::cloud_callback, this);

  raw_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      1,
      std::bind(&SGraphsNode::raw_odom_callback, this, std::placeholders::_1),
      sub_opt);

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      1024,
      std::bind(&SGraphsNode::imu_callback, this, std::placeholders::_1),
      sub_opt);

  room_data_sub = this->create_subscription<situational_graphs_msgs::msg::RoomsData>(
      "room_segmentation/room_data",
      10,
      std::bind(&SGraphsNode::room_data_callback, this, std::placeholders::_1),
      sub_opt);
  wall_data_sub = this->create_subscription<situational_graphs_msgs::msg::WallsData>(
      "wall_segmentation/wall_data",
      1,
      std::bind(&SGraphsNode::wall_data_callback, this, std::placeholders::_1),
      sub_opt);
  floor_data_sub = this->create_subscription<situational_graphs_msgs::msg::FloorData>(
      "floor_plan/floor_data",
      10,
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

  lidar_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "s_graphs/lidar_points", 1, pub_opt);
  map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "s_graphs/map_points", 1, pub_opt);
  wall_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "s_graphs/wall_points", 1, pub_opt);
  map_planes_pub = this->create_publisher<situational_graphs_msgs::msg::PlanesData>(
      "s_graphs/map_planes", 1, pub_opt);
  all_map_planes_pub = this->create_publisher<situational_graphs_msgs::msg::PlanesData>(
      "s_graphs/all_map_planes", 1, pub_opt);
  graph_pub = this->create_publisher<situational_graphs_reasoning_msgs::msg::Graph>(
      "s_graphs/graph_structure", 32, pub_opt);
  graph_keyframes_pub =
      this->create_publisher<situational_graphs_reasoning_msgs::msg::GraphKeyframes>(
          "s_graphs/graph_keyframes", 32, pub_opt);

  dump_service_server = this->create_service<situational_graphs_msgs::srv::DumpGraph>(
      "s_graphs/dump",
      std::bind(&SGraphsNode::dump_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  save_map_service_server = this->create_service<situational_graphs_msgs::srv::SaveMap>(
      "s_graphs/save_map",
      std::bind(&SGraphsNode::save_map_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  load_service_server = this->create_service<situational_graphs_msgs::srv::LoadGraph>(
      "s_graphs/load",
      std::bind(&SGraphsNode::load_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  loop_found = false;
  duplicate_planes_found = false;
  global_optimization = false;
  on_stairs = false;
  floor_node_updated = false;
  prev_edge_count = curr_edge_count = 0;
  prev_mapped_keyframes = 0;
  current_session_id = 0;
  prev_floor_level = -1;

  std::string optimization_type =
      this->get_parameter("optimization_type").get_parameter_value().get<std::string>();

  if (optimization_type == "GLOBAL") {
    ongoing_optimization_class = optimization_class::GLOBAL;
  } else if (optimization_type == "FLOOR_GLOBAL") {
    ongoing_optimization_class = optimization_class::FLOOR_GLOBAL;
  } else if (optimization_type == "LOCAL_GLOBAL") {
    ongoing_optimization_class = optimization_class::LOCAL_GLOBAL;
  }

  static_tf_timer =
      this->create_wall_timer(std::chrono::seconds(1),
                              std::bind(&SGraphsNode::publish_static_tfs, this),
                              callback_static_tf_timer);
}

void SGraphsNode::declare_ros_params() {
  this->declare_parameter("map_frame_id", "map");
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("fast_mapping", true);
  this->declare_parameter("viz_dense_map", false);
  this->declare_parameter("save_dense_map", false);
  this->declare_parameter("viz_all_floor_cloud", false);
  this->declare_parameter("map_cloud_resolution", 0.05);
  this->declare_parameter("map_cloud_pub_resolution", 0.1);
  this->declare_parameter("wait_trans_odom2map", false);

  this->declare_parameter("line_color_r", 0.0);
  this->declare_parameter("line_color_g", 0.0);
  this->declare_parameter("line_color_b", 0.0);

  this->declare_parameter("room_cube_color_r", 1.0);
  this->declare_parameter("room_cube_color_g", 0.07);
  this->declare_parameter("room_cube_color_b", 0.57);

  this->declare_parameter("floor_cube_color_r", 0.49);
  this->declare_parameter("floor_cube_color_g", 0.0);
  this->declare_parameter("floor_cube_color_b", 1.0);

  this->declare_parameter("line_marker_size", 0.04);
  this->declare_parameter("kf_marker_size", 0.3);
  this->declare_parameter("room_marker_size", 0.5);
  this->declare_parameter("floor_marker_size", 0.5);

  this->declare_parameter("marker_duration", 10);
  this->declare_parameter("save_timings", false);

  this->declare_parameter("plane_ransac_itr", 100);
  this->declare_parameter("plane_ransac_acc", 0.01);
  this->declare_parameter("plane_merging_tolerance", 0.35);

  this->declare_parameter("odom_pc_sync_queue", 32);
  this->declare_parameter("floor_level_viz_height", 10.0);
  this->declare_parameter("keyframe_viz_height", 0.0);
  this->declare_parameter("wall_viz_height", 0.0);
  this->declare_parameter("room_viz_height", 8.0);
  this->declare_parameter("floor_node_viz_height", 12.0);
  this->declare_parameter("use_floor_color_for_map", false);
  this->declare_parameter("always_publish_map", false);

  this->declare_parameter("max_keyframes_per_update", 10);
  this->declare_parameter("gps_time_offset", 0);
  this->declare_parameter("gps_edge_stddev_xy", 10000.0);
  this->declare_parameter("gps_edge_stddev_z", 10.0);

  this->declare_parameter("imu_time_offset", 0);
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
  this->declare_parameter("stand_still_time", 3.0);
  this->declare_parameter("stand_still_delta", 0.05);
  this->declare_parameter("record_kf_when_still", false);

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
  this->declare_parameter("cluster_min_size", 50);
  this->declare_parameter("cluster_clean_tolerance", 0.2);
  this->declare_parameter("cluster_seperation_tolerance", 2.0);

  this->declare_parameter("min_plane_points", 100);
  this->declare_parameter("min_plane_points_opti", 500);

  this->declare_parameter("infinite_room_information", 0.01);
  this->declare_parameter("infinite_room_dist_threshold", 1.0);
  this->declare_parameter("room_information", 0.01);
  this->declare_parameter("room_dist_threshold", 1.0);
  this->declare_parameter("dupl_plane_matching_information", 0.01);

  this->declare_parameter("enable_gps", false);
  this->declare_parameter("graph_update_interval", 3.0);
  this->declare_parameter("keyframe_timer_update_interval", 3.0);
  this->declare_parameter("map_cloud_update_interval", 3.0);
  this->declare_parameter("optimization_type", "GLOBAL");
}

void SGraphsNode::init_subclass() {
  covisibility_graph = std::make_shared<GraphSLAM>(
      this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
      this->get_parameter("save_timings").get_parameter_value().get<bool>());
  compressed_graph = std::make_unique<GraphSLAM>(
      this->get_parameter("g2o_solver_type").get_parameter_value().get<std::string>(),
      this->get_parameter("save_timings").get_parameter_value().get<bool>());
  visualization_graph = std::make_unique<GraphSLAM>();
  keyframe_updater = std::make_unique<KeyframeUpdater>(shared_from_this());
  plane_analyzer = std::make_unique<PlaneAnalyzer>(shared_from_this());
  loop_mapper = std::make_unique<LoopMapper>(shared_from_this(), graph_mutex);
  loop_detector = std::make_unique<LoopDetector>(shared_from_this(), graph_mutex);
  map_cloud_generator = std::make_unique<MapCloudGenerator>();
  inf_calclator = std::make_unique<InformationMatrixCalculator>(shared_from_this());
  nmea_parser = std::make_unique<NmeaSentenceParser>();
  plane_mapper = std::make_unique<PlaneMapper>(shared_from_this(), graph_mutex);
  inf_room_mapper =
      std::make_unique<InfiniteRoomMapper>(shared_from_this(), graph_mutex);
  finite_room_mapper =
      std::make_unique<FiniteRoomMapper>(shared_from_this(), graph_mutex);
  floor_mapper = std::make_unique<FloorMapper>(graph_mutex);
  graph_visualizer = std::make_unique<GraphVisualizer>(shared_from_this(), graph_mutex);
  keyframe_mapper = std::make_unique<KeyframeMapper>(shared_from_this(), graph_mutex);
  gps_mapper = std::make_unique<GPSMapper>(shared_from_this(), graph_mutex);
  imu_mapper = std::make_unique<IMUMapper>(shared_from_this(), graph_mutex);
  graph_publisher = std::make_unique<GraphPublisher>();
  wall_mapper = std::make_unique<WallMapper>(shared_from_this(), graph_mutex);
  room_graph_generator =
      std::make_unique<RoomGraphGenerator>(shared_from_this(), graph_mutex);

  main_timer->cancel();
}

void SGraphsNode::start_timers(bool enable_optimization_timer,
                               bool enable_keyframe_timer,
                               bool enable_map_publish_timer) {
  double graph_update_interval =
      this->get_parameter("graph_update_interval").get_parameter_value().get<double>();
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

  callback_static_tf_timer =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (enable_optimization_timer)
    optimization_timer = this->create_wall_timer(
        std::chrono::seconds(int(graph_update_interval)),
        std::bind(&SGraphsNode::optimization_timer_callback, this),
        callback_group_opt_timer);
  if (enable_keyframe_timer)
    keyframe_timer = this->create_wall_timer(
        std::chrono::seconds(int(keyframe_timer_update_interval)),
        std::bind(&SGraphsNode::keyframe_update_timer_callback, this),
        callback_keyframe_timer);
  if (enable_map_publish_timer) {
    bool pass = always_publish_map;
    map_publish_timer = this->create_wall_timer(
        std::chrono::seconds(int(map_cloud_update_interval)),
        [this, pass]() { this->map_publish_timer_callback(pass); },
        callback_map_pub_timer);
  }
}

void SGraphsNode::raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  Eigen::Isometry3d odom = odom2isometry(odom_msg);
  Eigen::Matrix4f odom_corrected;
  odom_corrected = trans_odom2map * odom.matrix().cast<float>();

  geometry_msgs::msg::PoseStamped pose_stamped_corrected;
  if (floors_vec.empty())
    pose_stamped_corrected =
        matrix2PoseStamped(odom_msg->header.stamp, odom_corrected, map_frame_id);
  else
    pose_stamped_corrected = matrix2PoseStamped(
        odom_msg->header.stamp,
        odom_corrected,
        "floor_" + std::to_string(floors_vec[current_floor_level].sequential_id) +
            "_layer");
  publish_corrected_odom(pose_stamped_corrected);

  geometry_msgs::msg::TransformStamped odom2map_transform = matrix2transform(
      odom_msg->header.stamp, trans_odom2map, map_frame_id, odom_frame_id);

  odom2map_broadcaster->sendTransform(odom2map_transform);
}

void SGraphsNode::init_map2odom_pose_callback(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
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

void SGraphsNode::cloud_callback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  const rclcpp::Time& stamp = cloud_msg->header.stamp;
  Eigen::Isometry3d odom = odom2isometry(odom_msg);

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // add the first floor node at keyframe height  before adding any keyframes
  if (keyframes.empty() && floors_vec.empty()) {
    add_first_floor_node();
  }

  Eigen::Matrix4f odom_corrected = trans_odom2map * odom.matrix().cast<float>();
  if (save_dense_map) keyframe_updater->augment_collected_cloud(odom_corrected, cloud);

  if (keyframe_updater->update(odom)) {
    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(
        stamp, odom, accum_d, cloud, current_floor_level, current_session_id));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    if (save_dense_map) {
      keyframe->set_dense_cloud(map_cloud_generator->generate_kf_cloud(
          current_floor_level,
          odom_corrected,
          keyframe_updater->get_collected_pose_cloud(),
          floors_vec,
          use_floor_color_for_map));
      keyframe_updater->reset_collected_pose_cloud();
    } else if (use_floor_color_for_map) {
      map_cloud_generator->color_cloud_using_floor_color(
          current_floor_level, floors_vec, cloud);
    }
    keyframe_queue.push_back(keyframe);
  }

  if (fast_mapping) {
    odom_cloud_queue_mutex.lock();
    odom_cloud_queue.push_back(std::make_pair(odom_corrected, cloud));
    odom_cloud_queue_mutex.unlock();
  }

  if (!floors_vec.empty()) {
    graph_mutex.lock();
    pcl::PointCloud<PointT>::Ptr cloud_trans =
        map_cloud_generator->generate(current_floor_level,
                                      odom_corrected,
                                      cloud,
                                      floors_vec,
                                      use_floor_color_for_map);
    graph_mutex.unlock();
    sensor_msgs::msg::PointCloud2 cloud_trans_msg;
    pcl::toROSMsg(*cloud_trans, cloud_trans_msg);

    cloud_trans_msg.header.frame_id =
        "floor_" + std::to_string(floors_vec[current_floor_level].sequential_id) +
        "_layer";
    cloud_trans_msg.header.stamp = cloud_msg->header.stamp;
    lidar_points_pub->publish(cloud_trans_msg);
  }
}

void SGraphsNode::room_data_callback(
    const situational_graphs_msgs::msg::RoomsData::SharedPtr rooms_msg) {
  std::lock_guard<std::mutex> lock(room_data_queue_mutex);
  room_data_queue.push_back(*rooms_msg);
}

void SGraphsNode::floor_data_callback(
    const situational_graphs_msgs::msg::FloorData::SharedPtr floor_data_msg) {
  std::lock_guard<std::mutex> lock(floor_data_mutex);
  floor_data_queue.push_back(*floor_data_msg);
}

void SGraphsNode::wall_data_callback(
    const situational_graphs_msgs::msg::WallsData::SharedPtr walls_msg) {
  for (size_t j = 0; j < walls_msg->walls.size(); j++) {
    std::vector<situational_graphs_msgs::msg::PlaneData> x_planes_msg =
        walls_msg->walls[j].x_planes;
    std::vector<situational_graphs_msgs::msg::PlaneData> y_planes_msg =
        walls_msg->walls[j].y_planes;

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
                             y_vert_planes,
                             walls_vec);
  }
}

void SGraphsNode::nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr nmea_msg) {
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

void SGraphsNode::navsat_callback(
    const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg) {
  geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg(
      new geographic_msgs::msg::GeoPointStamped());
  gps_msg->header = navsat_msg->header;
  gps_msg->position.latitude = navsat_msg->latitude;
  gps_msg->position.longitude = navsat_msg->longitude;
  gps_msg->position.altitude = navsat_msg->altitude;

  gps_callback(gps_msg);
}

void SGraphsNode::gps_callback(
    const geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg) {
  std::lock_guard<std::mutex> lock(gps_queue_mutex);
  rclcpp::Time(gps_msg->header.stamp) += rclcpp::Duration(gps_time_offset, 0);
  gps_queue.push_back(gps_msg);
}

void SGraphsNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  if (!enable_imu_orientation && !enable_imu_acceleration) {
    return;
  }

  std::lock_guard<std::mutex> lock(imu_queue_mutex);
  rclcpp::Time(imu_msg->header.stamp) += rclcpp::Duration(imu_time_offset, 0);
  imu_queue.push_back(imu_msg);
}

bool SGraphsNode::flush_keyframe_queue() {
  if (keyframe_queue.empty()) {
    // std::cout << "keyframe_queue is empty " << std::endl;
    return false;
  }

  trans_odom2map_mutex.lock();
  Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
  trans_odom2map_mutex.unlock();

  int num_processed = keyframe_mapper->map_keyframes(covisibility_graph,
                                                     odom2map,
                                                     keyframe_queue,
                                                     keyframes,
                                                     new_keyframes,
                                                     anchor_node,
                                                     anchor_edge,
                                                     keyframe_hash);

  graph_mutex.lock();
  if (floor_mapper->get_floor_level_update_info()) {
    std::vector<g2o::VertexPlane*> new_x_planes, new_y_planes;
    GraphUtils::update_node_floor_level(
        floors_vec.at(current_floor_level).stair_keyframe_ids.front(),
        current_floor_level,
        keyframes,
        x_vert_planes,
        y_vert_planes,
        rooms_vec,
        x_infinite_rooms,
        y_infinite_rooms,
        floors_vec,
        new_x_planes,
        new_y_planes,
        use_floor_color_for_map);
    GraphUtils::update_node_floor_level(current_floor_level, new_keyframes);
    plane_mapper->factor_new_planes(current_floor_level,
                                    covisibility_graph,
                                    new_x_planes,
                                    new_y_planes,
                                    x_vert_planes,
                                    y_vert_planes);
    if (on_stairs) on_stairs = false;
  }
  graph_mutex.unlock();

  // perform planar segmentation
  if (extract_planar_surfaces) {
    std::vector<std::thread> threads;
    for (long unsigned int i = 0; i < new_keyframes.size(); i++) {
      threads.push_back(std::thread([&, i]() {
        // Extract segmented planes
        std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec =
            plane_analyzer->extract_segmented_planes(new_keyframes[i]->cloud);

        // Map extracted planes, protected by graph_mutex
        plane_mapper->map_extracted_planes(covisibility_graph,
                                           new_keyframes[i],
                                           extracted_cloud_vec,
                                           x_vert_planes,
                                           y_vert_planes,
                                           hort_planes);
      }));
    }

    for (auto& t : threads) {
      if (t.joinable()) {
        t.join();
      }
    }
  }

  std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
  keyframe_queue.erase(keyframe_queue.begin(),
                       keyframe_queue.begin() + num_processed + 1);

  return true;
}

void SGraphsNode::flush_room_data_queue() {
  if (keyframes.empty() || floors_vec.empty()) {
    return;
  } else if (room_data_queue.empty()) {
    // std::cout << "room data queue is empty" << std::endl;
    return;
  }

  // update room height based on current_floor level
  for (auto& room_data_msg : room_data_queue) {
    for (auto& room_data : room_data_msg.rooms) {
      graph_mutex.lock();
      room_data.room_center.position.z =
          floors_vec[current_floor_level].node->estimate().translation().z();
      graph_mutex.unlock();
    }
  }

  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      dupl_y_vert_planes;
  for (const auto& room_data_msg : room_data_queue) {
    for (const auto& room_data : room_data_msg.rooms) {
      if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 2) {
        float x_width = PlaneUtils::width_between_planes(room_data.x_planes[0],
                                                         room_data.x_planes[1]);
        float y_width = PlaneUtils::width_between_planes(room_data.y_planes[0],
                                                         room_data.y_planes[1]);

        if (fabs(x_width) < 0.5 || fabs(y_width) < 0.5) continue;

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

        if (current_room_id != -1) {
          // generate local graph per room
          extract_keyframes_from_room(rooms_vec[current_room_id]);
          graph_mutex.lock();
          room_local_graph_id_queue.push_back(current_room_id);
          if (duplicate_planes_rooms) duplicate_planes_found = true;
          graph_mutex.unlock();
        }
      }
      // x infinite_room
      else if (room_data.x_planes.size() == 2 && room_data.y_planes.size() == 0) {
        float x_width = PlaneUtils::width_between_planes(room_data.x_planes[0],
                                                         room_data.x_planes[1]);
        if (fabs(x_width) < 0.5) continue;

        int current_room_id;
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
            rooms_vec,
            current_room_id);

        graph_mutex.lock();
        if (duplicate_planes_x_inf_rooms) duplicate_planes_found = true;
        graph_mutex.unlock();

      }
      // y infinite_room
      else if (room_data.x_planes.size() == 0 && room_data.y_planes.size() == 2) {
        float y_width = PlaneUtils::width_between_planes(room_data.y_planes[0],
                                                         room_data.y_planes[1]);
        if (fabs(y_width) < 0.5) continue;

        int current_room_id;
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
            rooms_vec,
            current_room_id);

        graph_mutex.lock();
        if (duplicate_planes_y_inf_rooms) duplicate_planes_found = true;
        graph_mutex.unlock();
      }
    }

    room_data_queue_mutex.lock();
    room_data_queue.pop_front();
    room_data_queue_mutex.unlock();
  }

  set_duplicate_planes(dupl_x_vert_planes, dupl_y_vert_planes);
  floor_mapper->factor_floor_room_nodes(covisibility_graph,
                                        floors_vec[current_floor_level],
                                        rooms_vec,
                                        x_infinite_rooms,
                                        y_infinite_rooms);
}

void SGraphsNode::set_duplicate_planes(
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_y_vert_planes) {
  graph_mutex.lock();
  for (const auto& dupl_x_plane : dupl_x_vert_planes) {
    x_vert_planes.find(dupl_x_plane.first.id)->second.duplicate_id =
        dupl_x_plane.second.id;

    x_vert_planes.find(dupl_x_plane.second.id)->second.duplicate_id =
        dupl_x_plane.first.id;
  }

  for (const auto& dupl_y_plane : dupl_y_vert_planes) {
    y_vert_planes.find(dupl_y_plane.first.id)->second.duplicate_id =
        dupl_y_plane.second.id;

    y_vert_planes.find(dupl_y_plane.second.id)->second.duplicate_id =
        dupl_y_plane.first.id;
  }
  graph_mutex.unlock();
}

void SGraphsNode::flush_floor_data_queue() {
  if (keyframes.empty()) {
    return;
  } else if (floor_data_queue.empty()) {
    // std::cout << "floor data queue is empty" << std::endl;
    return;
  }

  for (const auto& floor_data_msg : floor_data_queue) {
    if (floor_data_msg.state == situational_graphs_msgs::msg::FloorData::ON_STAIRS) {
      on_stairs = true;
      floor_data_mutex.lock();
      floor_data_queue.pop_front();
      floor_data_mutex.unlock();
      continue;
    } else {
      floor_mapper->lookup_floors(covisibility_graph,
                                  floor_data_msg,
                                  floors_vec,
                                  rooms_vec,
                                  x_infinite_rooms,
                                  y_infinite_rooms);

      // if new floor was added update floor level and add to it the stair keyframes
      if (!floor_data_msg.keyframe_ids.empty()) {
        current_floor_level = floor_mapper->get_floor_level();
        add_stair_keyframes_to_floor(floor_data_msg.keyframe_ids);
      }

      floor_data_mutex.lock();
      floor_data_queue.pop_front();
      floor_data_mutex.unlock();
    }
  }
}

bool SGraphsNode::flush_gps_queue() {
  std::lock_guard<std::mutex> lock(gps_queue_mutex);

  if (keyframes.empty() || gps_queue.empty()) {
    return false;
  }
  return gps_mapper->map_gps_data(covisibility_graph, gps_queue, keyframes);
}

bool SGraphsNode::flush_imu_queue() {
  std::lock_guard<std::mutex> lock(imu_queue_mutex);
  if (keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
    return false;
  }

  bool updated = false;
  imu_mapper->map_imu_data(
      covisibility_graph, tf_buffer, imu_queue, keyframes, base_frame_id);

  return updated;
}

void SGraphsNode::add_stair_keyframes_to_floor(
    const std::vector<int>& stair_keyframe_ids) {
  // get the keyframe ids and update their semantic of belonging to new floor
  floors_vec.at(current_floor_level).stair_keyframe_ids = stair_keyframe_ids;

  graph_mutex.lock();
  GraphUtils::set_stair_keyframes(floors_vec.at(current_floor_level).stair_keyframe_ids,
                                  keyframes);
  graph_mutex.unlock();
}

void SGraphsNode::add_first_floor_node() {
  situational_graphs_msgs::msg::FloorData floor_data_msg;
  floor_data_msg.floor_center.position.x = 0;
  floor_data_msg.floor_center.position.y = 0;
  floor_data_msg.floor_center.position.z = 0;

  floor_mapper->lookup_floors(covisibility_graph,
                              floor_data_msg,
                              floors_vec,
                              rooms_vec,
                              x_infinite_rooms,
                              y_infinite_rooms);
  current_floor_level = floor_mapper->get_floor_level();
}

void SGraphsNode::update_first_floor_node(const Eigen::Isometry3d& pose) {
  graph_mutex.lock();
  floors_vec.begin()->second.node->setEstimate(pose);
  graph_mutex.unlock();
  floor_node_updated = true;
}

void SGraphsNode::extract_keyframes_from_room(Rooms& current_room) {
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

void SGraphsNode::keyframe_update_timer_callback() {
  // add keyframes and floor coeffs in the queues to the pose graph
  bool keyframe_updated = flush_keyframe_queue();

  if (!keyframe_updated && !flush_gps_queue() && !flush_imu_queue()) {
    return;
  }

  // publish mapped planes
  publish_mapped_planes(x_vert_planes, y_vert_planes);

  // flush the room poses from room detector and no need to return if no rooms found
  flush_room_data_queue();

  // flush the floor poses from the floor planner and no need to return if no floors
  // found
  if (!keyframes.empty() && !floor_node_updated)
    update_first_floor_node(keyframes.begin()->second->estimate());
  flush_floor_data_queue();

  // loop detection
  if (!on_stairs) {
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes);
    if (loops.size() > 0) {
      loop_mapper->add_loops(covisibility_graph, loops);

      graph_mutex.lock();
      loop_found = true;
      graph_mutex.unlock();
    }
  } else {
    std::cout << "on stairs so not doing loop check " << std::endl;
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
    graph_mutex.lock();
    Eigen::Isometry3d anchor_target =
        static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
    anchor_node->setEstimate(anchor_target);
    graph_mutex.unlock();
  }

  publish_graph(covisibility_graph->graph.get(),
                keyframes,
                x_vert_planes,
                y_vert_planes,
                x_infinite_rooms,
                y_infinite_rooms,
                rooms_vec);
}

void SGraphsNode::optimization_timer_callback() {
  if (keyframes.empty() || floors_vec.empty()) return;

  int num_iterations =
      this->get_parameter("g2o_solver_num_iterations").get_parameter_value().get<int>();

  curr_edge_count = covisibility_graph->retrieve_total_nbr_of_edges();
  if (curr_edge_count == prev_edge_count) {
    return;
  }

  graph_mutex.lock();
  const int keyframe_id = keyframes.rbegin()->first;
  graph_mutex.unlock();

  switch (ongoing_optimization_class) {
    case optimization_class::GLOBAL: {
      handle_global_optimization();
      break;
    }

    case optimization_class::LOCAL_GLOBAL: {
      handle_local_global_optimization();
      break;
    }

    case optimization_class::FLOOR_GLOBAL: {
      handle_floor_global_optimization();
      break;
    }

    default:
      break;
  }

  // optimize the pose graph
  try {
    graph_mutex.lock();
    if (!global_optimization)
      compressed_graph->optimize("local", num_iterations);
    else {
      compressed_graph->optimize("global", num_iterations);
    }
    graph_mutex.unlock();
  } catch (std::invalid_argument& e) {
    std::cout << e.what() << std::endl;
    throw 1;
  }

  graph_mutex.lock();
  std::vector<int> updated_x_planes, updated_y_planes, updated_hort_planes;
  auto updated_planes_tuple =
      std::make_tuple(updated_x_planes, updated_y_planes, updated_hort_planes);

  GraphUtils::update_graph(compressed_graph,
                           keyframes,
                           x_vert_planes,
                           y_vert_planes,
                           hort_planes,
                           rooms_vec,
                           x_infinite_rooms,
                           y_infinite_rooms,
                           floors_vec,
                           updated_planes_tuple);

  if (global_optimization) {
    plane_mapper->convert_plane_points_to_map(
        x_vert_planes, y_vert_planes, hort_planes, updated_planes_tuple);
  }

  Eigen::Isometry3d trans =
      keyframes[keyframe_id]->node->estimate() * keyframes[keyframe_id]->odom.inverse();

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

  if (ongoing_optimization_class == optimization_class::LOCAL_GLOBAL ||
      ongoing_optimization_class == optimization_class::FLOOR_GLOBAL) {
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
  } else {
    graph_mutex.lock();
    room_local_graph_id_queue.clear();
    graph_mutex.unlock();
  }

  prev_edge_count = curr_edge_count;
}

void SGraphsNode::handle_global_optimization() {
  std::lock_guard<std::mutex> lock(graph_mutex);
  GraphUtils::copy_graph(covisibility_graph, compressed_graph, keyframes);
  global_optimization = true;
}

void SGraphsNode::handle_local_global_optimization() {
  std::lock_guard<std::mutex> lock(graph_mutex);

  if (!loop_found && !duplicate_planes_found) {
    GraphUtils::copy_windowed_graph(optimization_window_size,
                                    covisibility_graph,
                                    compressed_graph,
                                    keyframes,
                                    current_session_id);
    global_optimization = false;
  } else {
    GraphUtils::copy_graph(covisibility_graph, compressed_graph, keyframes);
    loop_found = false;
    duplicate_planes_found = false;
    global_optimization = true;
  }
}

void SGraphsNode::handle_floor_global_optimization() {
  std::lock_guard<std::mutex> lock(graph_mutex);

  if (!loop_found && !duplicate_planes_found) {
    GraphUtils::copy_windowed_graph(optimization_window_size,
                                    covisibility_graph,
                                    compressed_graph,
                                    keyframes,
                                    current_session_id);
    global_optimization = false;
  } else {
    GraphUtils::copy_floor_graph(current_floor_level,
                                 covisibility_graph,
                                 compressed_graph,
                                 keyframes,
                                 x_vert_planes,
                                 y_vert_planes,
                                 rooms_vec,
                                 x_infinite_rooms,
                                 y_infinite_rooms,
                                 walls_vec,
                                 floors_vec);
    duplicate_planes_found = false;
    loop_found = false;
    global_optimization = true;
  }
}

void SGraphsNode::broadcast_room_graph(const int room_id, const int num_iterations) {
  graph_mutex.lock();
  // optimize_room_local_graph
  rooms_vec[room_id].local_graph->optimize("room-local", num_iterations);
  GraphUtils::set_marginalize_info(rooms_vec[room_id].local_graph,
                                   covisibility_graph,
                                   rooms_vec[room_id].room_keyframes);
  graph_mutex.unlock();
}

void SGraphsNode::copy_data(
    std::vector<KeyFrame::Ptr>& kf_snapshot,
    std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
    std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
    std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
    std::unordered_map<int, InfiniteRooms>& x_inf_rooms_snapshot,
    std::unordered_map<int, InfiniteRooms>& y_inf_rooms_snapshot,
    std::unordered_map<int, Rooms>& rooms_vec_snapshot,
    std::map<int, Floors>& floors_vec_snapshot) {
  x_planes_snapshot = x_vert_planes;
  y_planes_snapshot = y_vert_planes;
  hort_planes_snapshot = hort_planes;
  x_inf_rooms_snapshot = x_infinite_rooms;
  y_inf_rooms_snapshot = y_infinite_rooms;
  rooms_vec_snapshot = rooms_vec;
  floors_vec_snapshot = floors_vec;

  kf_snapshot.resize(keyframes.size());
  std::transform(keyframes.begin(),
                 keyframes.end(),
                 kf_snapshot.begin(),
                 [=](const std::pair<int, KeyFrame::Ptr>& k) {
                   return std::make_shared<KeyFrame>(k.second);
                 });
}

void SGraphsNode::map_publish_timer_callback(bool pass) {
  if (keyframes.empty() || floors_vec.empty()) return;

  if (map_points_pub->get_subscription_count() == 0 &&
      wall_points_pub->get_subscription_count() == 0 &&
      markers_pub->get_subscription_count() == 0 && !pass) {
    return;
  }

  std::vector<KeyFrame::Ptr> kf_snapshot;
  std::unordered_map<int, VerticalPlanes> x_planes_snapshot, y_planes_snapshot;
  std::unordered_map<int, HorizontalPlanes> hort_planes_snapshot;
  std::unordered_map<int, InfiniteRooms> x_inf_rooms_snapshot, y_inf_rooms_snapshot;
  std::unordered_map<int, Rooms> rooms_vec_snapshot;
  std::map<int, Floors> floors_vec_snapshot;

  graph_mutex.lock();
  this->copy_data(kf_snapshot,
                  x_planes_snapshot,
                  y_planes_snapshot,
                  hort_planes_snapshot,
                  x_inf_rooms_snapshot,
                  y_inf_rooms_snapshot,
                  rooms_vec_snapshot,
                  floors_vec_snapshot);
  graph_mutex.unlock();
  auto current_time = this->now();

  graph_mutex.lock();
  int floor_level = current_floor_level;
  graph_mutex.unlock();

  std::unique_ptr<GraphSLAM> local_covisibility_graph;
  local_covisibility_graph = std::make_unique<GraphSLAM>("", false, false);
  graph_mutex.lock();
  GraphUtils::copy_entire_graph(covisibility_graph, local_covisibility_graph);
  graph_mutex.unlock();

  s_graphs_markers.markers.clear();
  s_graphs_markers = graph_visualizer->visualize_floor_covisibility_graph(
      current_time,
      local_covisibility_graph->graph.get(),
      kf_snapshot,
      x_planes_snapshot,
      y_planes_snapshot,
      hort_planes_snapshot,
      x_inf_rooms_snapshot,
      y_inf_rooms_snapshot,
      rooms_vec_snapshot,
      floors_vec_snapshot);

  std::unique_ptr<GraphSLAM> local_compressed_graph;
  local_compressed_graph = std::make_unique<GraphSLAM>("", false, false);
  bool is_optimization_global = false;
  graph_mutex.lock();
  GraphUtils::copy_graph_vertices(compressed_graph.get(), local_compressed_graph.get());
  is_optimization_global = global_optimization;
  graph_mutex.unlock();

  graph_visualizer->visualize_compressed_graph(current_time,
                                               floor_level,
                                               is_optimization_global,
                                               false,
                                               local_compressed_graph->graph.get(),
                                               kf_snapshot,
                                               x_planes_snapshot,
                                               y_planes_snapshot,
                                               hort_planes_snapshot,
                                               floors_vec_snapshot);

  markers_pub->publish(s_graphs_markers);
  publish_all_mapped_planes(x_planes_snapshot, y_planes_snapshot);

  sensor_msgs::msg::PointCloud2 s_graphs_cloud_msg;
  if (fast_mapping)
    handle_map_cloud(current_time,
                     floor_level,
                     odom_cloud_queue,
                     floors_vec_snapshot,
                     s_graphs_cloud_msg);
  else
    handle_map_cloud(current_time,
                     floor_level,
                     is_optimization_global,
                     prev_mapped_keyframes,
                     kf_snapshot,
                     floors_vec_snapshot,
                     s_graphs_cloud_msg,
                     prev_floor_level);
  map_points_pub->publish(s_graphs_cloud_msg);

  sensor_msgs::msg::PointCloud2 floor_wall_cloud_msg;
  if (fast_mapping)
    this->handle_floor_wall_cloud(current_time,
                                  floor_level,
                                  prev_mapped_keyframes,
                                  kf_snapshot,
                                  x_planes_snapshot,
                                  y_planes_snapshot,
                                  hort_planes_snapshot,
                                  floor_wall_cloud_msg);
  else
    this->handle_floor_wall_cloud(floor_level,
                                  x_planes_snapshot,
                                  y_planes_snapshot,
                                  hort_planes_snapshot,
                                  floors_vec_snapshot,
                                  floor_wall_cloud_msg);
  wall_points_pub->publish(floor_wall_cloud_msg);

  // copy floor cloud to floors_vec
  graph_mutex.lock();
  if (is_optimization_global) {
    auto it = floors_vec_snapshot.find(floor_level);
    for (; it != floors_vec_snapshot.end(); ++it) {
      floors_vec[it->first].floor_cloud = floors_vec_snapshot[it->first].floor_cloud;
    }
  } else
    floors_vec[floor_level].floor_cloud = floors_vec_snapshot[floor_level].floor_cloud;
  floors_vec[floor_level].floor_wall_cloud =
      floors_vec_snapshot[floor_level].floor_wall_cloud;
  graph_mutex.unlock();

  prev_mapped_keyframes = kf_snapshot.size();
  prev_floor_level = floor_level;
}

void SGraphsNode::handle_map_cloud(
    const rclcpp::Time& current_time,
    int floor_level,
    std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>&
        odom_cloud_queue,
    std::map<int, Floors> floors_vec_snapshot,
    sensor_msgs::msg::PointCloud2& s_graphs_cloud_msg) {
  std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>
      current_odom_cloud_queue;

  odom_cloud_queue_mutex.lock();
  current_odom_cloud_queue = odom_cloud_queue;
  odom_cloud_queue_mutex.unlock();

  Eigen::Matrix4f map_floor_t(Eigen::Matrix4f::Identity());
  if (!floors_vec_snapshot.empty()) {
    geometry_msgs::msg::TransformStamped map_floor_tranform_stamped;
    try {
      map_floor_tranform_stamped = tf_buffer->lookupTransform(
          map_frame_id,
          "floor_" + std::to_string(floors_vec_snapshot[floor_level].sequential_id) +
              "_layer",
          tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      return;
    }
    map_floor_t = transformStamped2EigenMatrix(map_floor_tranform_stamped);
  }

  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
  for (const auto& odom_cloud_pair : current_odom_cloud_queue)
    *cloud_transformed +=
        *map_cloud_generator->generate(floor_level,
                                       map_floor_t * odom_cloud_pair.first,
                                       odom_cloud_pair.second,
                                       floors_vec_snapshot,
                                       use_floor_color_for_map);
  pcl::toROSMsg(*cloud_transformed, s_graphs_cloud_msg);
  s_graphs_cloud_msg.header.stamp = current_time;
  s_graphs_cloud_msg.header.frame_id = map_frame_id;

  std::lock_guard<std::mutex> lock(odom_cloud_queue_mutex);
  odom_cloud_queue.erase(odom_cloud_queue.begin(),
                         odom_cloud_queue.begin() + current_odom_cloud_queue.size());
}

void SGraphsNode::handle_map_cloud(const rclcpp::Time& current_time,
                                   int floor_level,
                                   bool is_optimization_global,
                                   int prev_mapped_kfs,
                                   const std::vector<KeyFrame::Ptr>& kf_snapshot,
                                   std::map<int, Floors>& floors_vec_snapshot,
                                   sensor_msgs::msg::PointCloud2& s_graphs_cloud_msg,
                                   const int prev_floor_level) {
  if (floors_vec_snapshot.empty()) return;

  size_t kfs_to_map = kf_snapshot.size() - prev_mapped_kfs;

  Eigen::Matrix4f map_floor_t =
      get_floor_map_transform(floors_vec_snapshot[floor_level]);

  if (floors_vec_snapshot[floor_level].floor_cloud->points.empty() ||
      is_optimization_global) {
    auto it = floors_vec_snapshot.begin();
    if (prev_floor_level != floor_level) {
      it = floors_vec_snapshot.find(prev_floor_level);
    } else {
      it = floors_vec_snapshot.find(floor_level);
    }
    for (; it != floors_vec_snapshot.end(); ++it) {
      Eigen::Matrix4f map_floor_t = get_floor_map_transform(it->second);
      floors_vec_snapshot[it->first].floor_cloud =
          map_cloud_generator->generate_floor_cloud(kf_snapshot,
                                                    it->first,
                                                    map_cloud_pub_resolution,
                                                    map_floor_t,
                                                    viz_dense_map && save_dense_map);
    }
  } else if (kfs_to_map != 0) {
    std::vector<KeyFrame::Ptr> kf_map_window;
    for (auto it = kf_snapshot.rbegin();
         it != kf_snapshot.rend() && kf_map_window.size() <= kfs_to_map;
         ++it) {
      kf_map_window.push_back(*it);
    }
    pcl::PointCloud<PointT>::Ptr augmented_cloud =
        map_cloud_generator->generate_floor_cloud(kf_map_window,
                                                  floor_level,
                                                  map_cloud_pub_resolution,
                                                  map_floor_t,
                                                  viz_dense_map && save_dense_map);

    *floors_vec_snapshot[floor_level].floor_cloud += *augmented_cloud;
  }

  if (floors_vec_snapshot[floor_level].floor_cloud == nullptr) return;

  pcl::PointCloud<PointT> map_cloud;
  pcl::copyPointCloud(*floors_vec_snapshot[floor_level].floor_cloud, map_cloud);

  if (viz_all_floor_cloud) {
    for (const auto& floor : floors_vec_snapshot) {
      if (floor.first != floor_level) {
        map_cloud += *floor.second.floor_cloud;
      }
    }
  }

  pcl::toROSMsg(map_cloud, s_graphs_cloud_msg);
  s_graphs_cloud_msg.header.stamp = current_time;
  s_graphs_cloud_msg.header.frame_id = map_frame_id;
}

Eigen::Matrix4f SGraphsNode::get_floor_map_transform(const Floors floor) {
  Eigen::Matrix4f map_floor_t(Eigen::Matrix4f::Identity());
  geometry_msgs::msg::TransformStamped map_floor_tranform_stamped;
  try {
    map_floor_tranform_stamped = tf_buffer->lookupTransform(
        map_frame_id,
        "floor_" + std::to_string(floor.sequential_id) + "_layer",
        tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    return map_floor_t;
  }
  map_floor_t = transformStamped2EigenMatrix(map_floor_tranform_stamped);

  return map_floor_t;
}

void SGraphsNode::handle_floor_wall_cloud(
    const int& floor_level,
    const std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
    std::map<int, Floors>& floors_vec_snapshot,
    sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg) {
  pcl::PointCloud<PointNormal>::Ptr floor_wall_cloud(new pcl::PointCloud<PointNormal>);

  for (const auto& x_plane : x_planes_snapshot) {
    if (x_plane.second.floor_level != floor_level) continue;

    *floor_wall_cloud += *x_plane.second.cloud_seg_map;
  }

  for (const auto& y_plane : y_planes_snapshot) {
    if (y_plane.second.floor_level != floor_level) continue;
    *floor_wall_cloud += *y_plane.second.cloud_seg_map;
  }

  for (const auto& hort_plane : hort_planes_snapshot) {
    if (hort_plane.second.floor_level != floor_level) continue;
    *floor_wall_cloud += *hort_plane.second.cloud_seg_map;
  }

  floors_vec_snapshot[floor_level].floor_wall_cloud = floor_wall_cloud;

  pcl::toROSMsg(*floors_vec_snapshot[floor_level].floor_wall_cloud,
                floor_wall_cloud_msg);
  floor_wall_cloud_msg = transform_floor_cloud(
      floor_wall_cloud_msg,
      map_frame_id,
      "floor_" + std::to_string(floors_vec_snapshot[floor_level].sequential_id) +
          "_walls_layer");

  this->concatenate_floor_wall_clouds(
      floor_level, floor_wall_cloud_msg, floors_vec_snapshot);
}

void SGraphsNode::concatenate_floor_wall_clouds(
    const int& floor_level,
    sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg,
    const std::map<int, Floors>& floors_vec_snapshot) {
  sensor_msgs::msg::PointCloud2 current_floor_wall_cloud_msg;
  for (auto& floor : floors_vec_snapshot) {
    if (floor.second.id == floor_level ||
        floor.second.floor_wall_cloud->points.empty()) {
      continue;
    }

    pcl::toROSMsg(*floor.second.floor_wall_cloud, current_floor_wall_cloud_msg);
    current_floor_wall_cloud_msg = transform_floor_cloud(
        current_floor_wall_cloud_msg,
        map_frame_id,
        "floor_" + std::to_string(floor.second.sequential_id) + "_walls_layer");
    pcl::concatenatePointCloud(
        floor_wall_cloud_msg, current_floor_wall_cloud_msg, floor_wall_cloud_msg);
  }
}

void SGraphsNode::handle_floor_wall_cloud(
    const rclcpp::Time& current_time,
    const int& floor_level,
    const int& prev_mapped_kfs,
    const std::vector<KeyFrame::Ptr>& kf_snapshot,
    const std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
    sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg) {
  size_t kfs_to_map = kf_snapshot.size() - prev_mapped_kfs;
  std::vector<KeyFrame::Ptr> kf_map_window;
  pcl::PointCloud<PointNormal>::Ptr wall_map_cloud(new pcl::PointCloud<PointNormal>());

  if (kfs_to_map != 0) {
    for (auto it = kf_snapshot.rbegin();
         it != kf_snapshot.rend() && kf_map_window.size() <= kfs_to_map;
         ++it) {
      kf_map_window.push_back(*it);
    }
  } else
    return;

  for (const auto& kf : kf_map_window) {
    for (const auto& x_id : kf->x_plane_ids) {
      auto x_plane = x_planes_snapshot.find(x_id);
      this->update_wall_cloud<VerticalPlanes>(
          kf, floor_level, x_plane->second, wall_map_cloud);
    }
    for (const auto& y_id : kf->y_plane_ids) {
      auto y_plane = y_planes_snapshot.find(y_id);
      this->update_wall_cloud<VerticalPlanes>(
          kf, floor_level, y_plane->second, wall_map_cloud);
    }
    for (const auto& h_id : kf->hort_plane_ids) {
      auto h_plane = hort_planes_snapshot.find(h_id);
      this->update_wall_cloud<HorizontalPlanes>(
          kf, floor_level, h_plane->second, wall_map_cloud);
    }
  }

  pcl::toROSMsg(*wall_map_cloud, floor_wall_cloud_msg);
  floor_wall_cloud_msg.header.stamp = current_time;
  floor_wall_cloud_msg.header.frame_id = map_frame_id;
}

template <typename planeT>
void SGraphsNode::update_wall_cloud(const KeyFrame::Ptr kf,
                                    const int& floor_level,
                                    const planeT plane,
                                    pcl::PointCloud<PointNormal>::Ptr& wall_map_cloud) {
  int search_id = kf->id();
  auto current_kf = std::find_if(plane.keyframe_node_vec.begin(),
                                 plane.keyframe_node_vec.end(),
                                 [search_id](const g2o::VertexSE3* keyframe) {
                                   return keyframe->id() == search_id;
                                 });

  if (current_kf != plane.keyframe_node_vec.end()) {
    int kf_position = std::distance(plane.keyframe_node_vec.begin(), current_kf);
    auto kf_cloud_body = plane.cloud_seg_body_vec[kf_position];

    Eigen::Matrix4f map_floor_t(Eigen::Matrix4f::Identity());
    if (!floors_vec.empty()) {
      geometry_msgs::msg::TransformStamped map_floor_tranform_stamped;
      try {
        map_floor_tranform_stamped = tf_buffer->lookupTransform(
            map_frame_id,
            "floor_" + std::to_string(floors_vec[floor_level].sequential_id) + "_layer",
            tf2::TimePointZero);
      } catch (tf2::TransformException& ex) {
        return;
      }

      map_floor_t = transformStamped2EigenMatrix(map_floor_tranform_stamped);
    }

    for (const auto& src_pt : kf_cloud_body->points) {
      // TODO: Check if mutex if needed
      PointNormal dst_pt;
      dst_pt.r = plane.color[0];
      dst_pt.g = plane.color[1];
      dst_pt.b = plane.color[2];

      dst_pt.getVector4fMap() = map_floor_t *
                                kf->node->estimate().matrix().cast<float>() *
                                src_pt.getVector4fMap();
      wall_map_cloud->push_back(dst_pt);
    }
  }
}

void SGraphsNode::publish_graph(
    g2o::SparseOptimizer* local_covisibility_graph,
    std::map<int, KeyFrame::Ptr> keyframes_complete_snapshot,
    std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
    std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
    std::unordered_map<int, InfiniteRooms>& x_inf_rooms_snapshot,
    std::unordered_map<int, InfiniteRooms>& y_inf_rooms_snapshot,
    std::unordered_map<int, Rooms>& rooms_vec_snapshot) {
  std::string graph_type;
  if (std::string("/robot1") == this->get_namespace()) {
    graph_type = "Prior";
  } else {
    graph_type = "Online";
  }

  graph_mutex.lock();
  auto graph_structure = graph_publisher->publish_graph(local_covisibility_graph,
                                                        "Online",
                                                        x_vert_planes_prior,
                                                        y_vert_planes_prior,
                                                        rooms_vec_prior,
                                                        x_planes_snapshot,
                                                        y_planes_snapshot,
                                                        rooms_vec_snapshot,
                                                        x_inf_rooms_snapshot,
                                                        y_inf_rooms_snapshot);
  graph_mutex.unlock();
  graph_structure.name = graph_type;

  graph_mutex.lock();
  auto graph_keyframes = graph_publisher->publish_graph_keyframes(
      local_covisibility_graph, keyframes_complete_snapshot, dump_directory);
  graph_mutex.unlock();

  graph_pub->publish(graph_structure);
  graph_keyframes_pub->publish(graph_keyframes);
}

sensor_msgs::msg::PointCloud2 SGraphsNode::transform_floor_cloud(
    const sensor_msgs::msg::PointCloud2 input_floor_cloud,
    const std::string target_frame_id,
    const std::string source_frame_id) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer->lookupTransform(
        target_frame_id, source_frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    std::cout << "could not find transform between " << target_frame_id << " and"
              << source_frame_id << std::endl;
    return input_floor_cloud;
  }

  return apply_transform(input_floor_cloud, transform_stamped);
}

inline sensor_msgs::msg::PointCloud2 SGraphsNode::apply_transform(
    const sensor_msgs::msg::PointCloud2 input_floor_cloud,
    const geometry_msgs::msg::TransformStamped transform_stamped) {
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  try {
    tf2::doTransform(input_floor_cloud, transformed_pointcloud, transform_stamped);
  } catch (tf2::TransformException& ex) {
    std::cout << "Could not transform point cloud " << ex.what() << std::endl;
  }

  return transformed_pointcloud;
}

void SGraphsNode::publish_mapped_planes(
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
      unique_x_plane_ids.insert(std::pair<int, int>(x_plane_id, 1));
    }

    for (const auto& y_plane_id : (it)->second->y_plane_ids) {
      unique_y_plane_ids.insert(std::pair<int, int>(y_plane_id, 1));
    }
  }

  situational_graphs_msgs::msg::PlanesData vert_planes_data;
  vert_planes_data.header.stamp = keyframes.rbegin()->second->stamp;
  for (const auto& unique_x_plane_id : unique_x_plane_ids) {
    auto local_x_vert_plane = x_vert_planes_snapshot.find(unique_x_plane_id.first);

    if (local_x_vert_plane == x_vert_planes_snapshot.end() ||
        local_x_vert_plane->second.floor_level != current_floor_level)
      continue;
    situational_graphs_msgs::msg::PlaneData plane_data;
    Eigen::Vector4d mapped_plane_coeffs;
    graph_mutex.lock();
    mapped_plane_coeffs = (local_x_vert_plane->second).plane_node->estimate().coeffs();
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
    graph_mutex.unlock();
    vert_planes_data.x_planes.push_back(plane_data);
  }

  for (const auto& unique_y_plane_id : unique_y_plane_ids) {
    auto local_y_vert_plane = y_vert_planes_snapshot.find(unique_y_plane_id.first);

    if (local_y_vert_plane == y_vert_planes_snapshot.end() ||
        local_y_vert_plane->second.floor_level != current_floor_level)
      continue;
    situational_graphs_msgs::msg::PlaneData plane_data;
    Eigen::Vector4d mapped_plane_coeffs;
    graph_mutex.lock();
    mapped_plane_coeffs = (local_y_vert_plane->second).plane_node->estimate().coeffs();
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
    graph_mutex.unlock();

    vert_planes_data.y_planes.push_back(plane_data);
  }
  map_planes_pub->publish(vert_planes_data);
}

void SGraphsNode::publish_all_mapped_planes(
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes_snapshot) {
  if (keyframes.empty()) return;

  // int current_floor_level = floor_mapper->get_floor_level();

  situational_graphs_msgs::msg::PlanesData vert_planes_data;
  vert_planes_data.header.stamp = keyframes.rbegin()->second->stamp;
  for (const auto& x_vert_plane : x_vert_planes_snapshot) {
    if (x_vert_plane.second.floor_level != current_floor_level) continue;

    situational_graphs_msgs::msg::PlaneData plane_data;
    Eigen::Vector4d mapped_plane_coeffs;
    graph_mutex.lock();
    mapped_plane_coeffs = (x_vert_plane).second.plane_node->estimate().coeffs();
    // correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE,
    // mapped_plane_coeffs);
    plane_data.id = (x_vert_plane).second.id;
    plane_data.nx = mapped_plane_coeffs(0);
    plane_data.ny = mapped_plane_coeffs(1);
    plane_data.nz = mapped_plane_coeffs(2);
    plane_data.d = mapped_plane_coeffs(3);
    if (x_vert_plane.second.type == "Prior") {
      plane_data.data_source = "PRIOR";
    } else {
      plane_data.data_source = "Online";
    }
    for (const auto& plane_point_data : (x_vert_plane).second.cloud_seg_map->points) {
      geometry_msgs::msg::Vector3 plane_point;
      plane_point.x = plane_point_data.x;
      plane_point.y = plane_point_data.y;
      plane_point.z = plane_point_data.z;
      plane_data.plane_points.push_back(plane_point);
    }
    graph_mutex.unlock();
    vert_planes_data.x_planes.push_back(plane_data);
  }

  for (const auto& y_vert_plane : y_vert_planes_snapshot) {
    if (y_vert_plane.second.floor_level != current_floor_level) continue;

    situational_graphs_msgs::msg::PlaneData plane_data;
    Eigen::Vector4d mapped_plane_coeffs;
    graph_mutex.lock();
    mapped_plane_coeffs = (y_vert_plane).second.plane_node->estimate().coeffs();
    // correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE,
    // mapped_plane_coeffs);
    plane_data.id = (y_vert_plane).second.id;
    plane_data.nx = mapped_plane_coeffs(0);
    plane_data.ny = mapped_plane_coeffs(1);
    plane_data.nz = mapped_plane_coeffs(2);
    plane_data.d = mapped_plane_coeffs(3);
    if (y_vert_plane.second.type == "Prior") {
      plane_data.data_source = "PRIOR";
    } else {
      plane_data.data_source = "Online";
    }
    for (const auto& plane_point_data : (y_vert_plane).second.cloud_seg_map->points) {
      geometry_msgs::msg::Vector3 plane_point;
      plane_point.x = plane_point_data.x;
      plane_point.y = plane_point_data.y;
      plane_point.z = plane_point_data.z;
      plane_data.plane_points.push_back(plane_point);
    }
    graph_mutex.unlock();
    vert_planes_data.y_planes.push_back(plane_data);
  }
  all_map_planes_pub->publish(vert_planes_data);
}

void SGraphsNode::publish_corrected_odom(
    geometry_msgs::msg::PoseStamped pose_stamped_corrected) {
  nav_msgs::msg::Path path_stamped_corrected;
  path_stamped_corrected.header = pose_stamped_corrected.header;
  odom_path_vec.push_back(pose_stamped_corrected);
  path_stamped_corrected.poses = odom_path_vec;

  odom_pose_corrected_pub->publish(pose_stamped_corrected);
  odom_path_corrected_pub->publish(path_stamped_corrected);
}

void SGraphsNode::publish_static_tfs() {
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

  graph_mutex.lock();
  std::map<int, Floors> floors_vec_snapshot = floors_vec;
  graph_mutex.unlock();

  auto current_time = this->get_clock()->now();
  for (auto floor = floors_vec_snapshot.begin(); floor != floors_vec_snapshot.end();
       ++floor) {
    double floor_height;
    if (floor->second.sequential_id == 0)
      floor_height = 0;
    else
      floor_height = floor_level_viz_height;
    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();

    if (floor->second.sequential_id != 0) {
      graph_mutex.lock();
      double floor_z_diff =
          floor->second.node->estimate().translation().z() -
          floors_vec_snapshot.begin()->second.node->estimate().translation().z();
      graph_mutex.unlock();

      floor_height = floor_z_diff * floor_height;
    }

    // map to floor transform
    geometry_msgs::msg::TransformStamped map_floor_transform;
    map_floor_transform.header.stamp = current_time;
    map_floor_transform.header.frame_id = map_frame_id;
    map_floor_transform.child_frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_layer";
    map_floor_transform.transform.translation.x = 0;
    map_floor_transform.transform.translation.y = 0;
    map_floor_transform.transform.translation.z = floor_height;
    map_floor_transform.transform.rotation = transform.transform.rotation;
    static_transforms.push_back(map_floor_transform);

    // floor to keyframe transform
    geometry_msgs::msg::TransformStamped floor_keyframe_transform;
    floor_keyframe_transform.header.stamp = current_time;
    floor_keyframe_transform.header.frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_layer";
    floor_keyframe_transform.child_frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_keyframes_layer";
    floor_keyframe_transform.transform.translation.x = 0;
    floor_keyframe_transform.transform.translation.y = 0;
    floor_keyframe_transform.transform.translation.z = keyframe_viz_height;
    floor_keyframe_transform.transform.rotation = transform.transform.rotation;
    static_transforms.push_back(floor_keyframe_transform);

    // floor to walls transform
    geometry_msgs::msg::TransformStamped floor_wall_transform;
    floor_wall_transform.header.stamp = current_time;
    floor_wall_transform.header.frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_layer";
    floor_wall_transform.child_frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_walls_layer";
    floor_wall_transform.transform.translation.x = 0;
    floor_wall_transform.transform.translation.y = 0;
    floor_wall_transform.transform.translation.z = wall_viz_height;
    floor_wall_transform.transform.rotation = transform.transform.rotation;
    static_transforms.push_back(floor_wall_transform);

    // floor to rooms transform
    geometry_msgs::msg::TransformStamped floor_room_transform;
    floor_room_transform.header.stamp = current_time;
    floor_room_transform.header.frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_layer";
    floor_room_transform.child_frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_rooms_layer";
    floor_room_transform.transform.translation.x = 0;
    floor_room_transform.transform.translation.y = 0;
    floor_room_transform.transform.translation.z = room_viz_height;
    floor_room_transform.transform.rotation = transform.transform.rotation;
    static_transforms.push_back(floor_room_transform);

    // floor to floor transform
    geometry_msgs::msg::TransformStamped floor_floor_transform;
    floor_floor_transform.header.stamp = current_time;
    floor_floor_transform.header.frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_layer";
    floor_floor_transform.child_frame_id =
        "floor_" + std::to_string(floor->second.sequential_id) + "_floors_layer";
    floor_floor_transform.transform.translation.x = 0;
    floor_floor_transform.transform.translation.y = 0;
    floor_floor_transform.transform.translation.z = floor_node_viz_height;
    floor_floor_transform.transform.rotation = transform.transform.rotation;
    static_transforms.push_back(floor_floor_transform);
  }

  for (const auto& transform : static_transforms) {
    floor_map_static_transforms->sendTransform(transform);
  }
}

void SGraphsNode::set_dump_directory() {
  dump_directory = "/tmp/s_graphs_data";
  std::time_t t = std::time(nullptr);
  struct std::tm* time_now = std::localtime(&t);
  std::stringstream ss;
  ss << (time_now->tm_year + 1900) << '-' << (time_now->tm_mon + 1) << '_'
     << time_now->tm_mday << '_' << time_now->tm_hour << '_' << time_now->tm_min << "_"
     << time_now->tm_sec;
  dump_directory += "_" + ss.str();
  std::cout << "Dump folder name is: " << dump_directory << std::endl;
}

std::string SGraphsNode::move_directory_to_new_destination(
    const std::string& old_directory,
    const std::string& new_destination) {
  boost::filesystem::path old_path(old_directory);

  if (!boost::filesystem::exists(old_path)) {
    boost::filesystem::create_directory(old_directory);
  }

  boost::filesystem::path new_path(new_destination + "/" +
                                   old_path.filename().string());

  try {
    boost::filesystem::rename(old_path, new_path);
    std::cout << "Directory moved successfully to: " << new_path.string() << std::endl;
    return new_path.string();
  } catch (const boost::filesystem::filesystem_error& e) {
    std::cerr << "Error moving directory: " << e.what() << std::endl;
    return "";
  }
}

bool SGraphsNode::dump_service(
    const std::shared_ptr<situational_graphs_msgs::srv::DumpGraph::Request> req,
    std::shared_ptr<situational_graphs_msgs::srv::DumpGraph::Response> res) {
  std::string req_directory = req->destination;
  std::string new_dump_directory;
  if (req_directory == "") {
    new_dump_directory = dump_directory;
  } else {
    new_dump_directory =
        move_directory_to_new_destination(dump_directory, req_directory);
    if (new_dump_directory == "") {
      std::cout << "Dump directory not created properly, not saving data " << std::endl;
      return false;
    }
  }

  if (floors_vec[current_floor_level].floor_cloud->points.empty()) {
    map_publish_timer_callback(true);
  }

  std::lock_guard<std::mutex> lock(graph_mutex);
  if (!boost::filesystem::is_directory(new_dump_directory)) {
    boost::filesystem::create_directory(new_dump_directory);
  }

  std::string kf_directory = new_dump_directory + "/keyframes";
  if (!boost::filesystem::is_directory(kf_directory)) {
    boost::filesystem::create_directory(kf_directory);
  }
  std::string x_vert_planes_directory = new_dump_directory + "/x_vert_planes";
  if (!boost::filesystem::is_directory(x_vert_planes_directory)) {
    boost::filesystem::create_directory(x_vert_planes_directory);
  }
  std::string y_vert_planes_directory = new_dump_directory + "/y_vert_planes";
  if (!boost::filesystem::is_directory(y_vert_planes_directory)) {
    boost::filesystem::create_directory(y_vert_planes_directory);
  }
  std::string hort_planes_directory = new_dump_directory + "/hort_planes";
  if (!boost::filesystem::is_directory(hort_planes_directory)) {
    boost::filesystem::create_directory(hort_planes_directory);
  }
  std::string walls_directory = new_dump_directory + "/walls";
  if (!boost::filesystem::is_directory(walls_directory)) {
    boost::filesystem::create_directory(walls_directory);
  }
  std::string rooms_directory = new_dump_directory + "/rooms";
  if (!boost::filesystem::is_directory(rooms_directory)) {
    boost::filesystem::create_directory(rooms_directory);
  }
  std::string floors_directory = new_dump_directory + "/floors";
  if (!boost::filesystem::is_directory(floors_directory)) {
    boost::filesystem::create_directory(floors_directory);
  }

  std::cout << "All data will be dumped to: " << new_dump_directory << std::endl;
  covisibility_graph->save(new_dump_directory + "/graph.g2o");

  int id = 0;
  for (const auto& kf : keyframes) {
    kf.second->save(kf_directory, id);
    id++;
  }

  id = 0;
  for (auto& x_vert_plane : x_vert_planes) {
    x_vert_plane.second.save(x_vert_planes_directory, 'x', id);
    id++;
  }

  id = 0;
  for (auto& y_vert_plane : y_vert_planes) {
    y_vert_plane.second.save(y_vert_planes_directory, 'y', id);
    id++;
  }

  id = 0;
  for (auto& hort_plane : hort_planes) {
    hort_plane.second.save(hort_planes_directory, 'h', id);
    id++;
  }

  id = 0;
  for (auto& wall : walls_vec) {
    if (wall.second.plane_type == PlaneUtils::plane_class::X_VERT_PLANE)
      wall.second.save(walls_directory, x_vert_planes, id);
    else if (wall.second.plane_type == PlaneUtils::plane_class::Y_VERT_PLANE)
      wall.second.save(walls_directory, y_vert_planes, id);
    id++;
  }

  id = 0;
  for (auto& room : rooms_vec) {
    room.second.save(rooms_directory, id);
    id++;
  }

  id = 0;
  for (auto& floor : floors_vec) {
    floor.second.save(floors_directory, id);
    id++;
  }

  if (zero_utm) {
    std::ofstream zero_utm_ofs(new_dump_directory + "/zero_utm");
    zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() %
                        zero_utm->z()
                 << std::endl;
    zero_utm_ofs.close();
  }

  std::ofstream anchor_ofs(new_dump_directory + "/anchor_node.txt");
  if (anchor_node != nullptr) {
    anchor_ofs << "id " << anchor_node->id() << "\n";
    anchor_ofs << "estimate " << anchor_node->estimate().matrix() << "\n";
  }
  anchor_ofs.close();

  std::ofstream session_ofs(new_dump_directory + "/session_details.txt");
  session_ofs << "session_id " << current_session_id << "\n";
  session_ofs << "session_graph_vertices "
              << covisibility_graph->retrieve_total_nbr_of_vertices() << "\n";
  session_ofs << "session_graph_edges "
              << covisibility_graph->retrieve_total_nbr_of_edges() << "\n";

  session_ofs.close();

  res->success = true;
  return true;
}

bool SGraphsNode::save_map_service(
    const std::shared_ptr<situational_graphs_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<situational_graphs_msgs::srv::SaveMap::Response> res) {
  if (keyframes.empty()) {
    res->success = false;
    return true;
  }

  std::vector<KeyFrame::Ptr> kf_snapshot;
  graph_mutex.lock();
  kf_snapshot.resize(keyframes.size());
  std::transform(keyframes.begin(),
                 keyframes.end(),
                 kf_snapshot.begin(),
                 [=](const std::pair<int, KeyFrame::Ptr>& k) {
                   return std::make_shared<KeyFrame>(k.second);
                 });
  graph_mutex.unlock();

  auto cloud = map_cloud_generator->generate(
      kf_snapshot, req->resolution, Eigen::Matrix4f::Identity(), save_dense_map);
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
  cloud->header.stamp = kf_snapshot.back()->cloud->header.stamp;

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

bool SGraphsNode::load_service(
    const std::shared_ptr<situational_graphs_msgs::srv::LoadGraph::Request> req,
    std::shared_ptr<situational_graphs_msgs::srv::LoadGraph::Response> res) {
  std::string directory = req->destination;
  std::cout << "Reading data from: " << directory << std::endl;

  std::vector<std::string> keyframe_directories, y_planes_directories,
      x_planes_directories, hort_plane_directories, wall_directories, room_directories,
      floor_directories;

  std::map<int, KeyFrame::Ptr> loaded_keyframes;

  // read the session id
  std::ifstream ifs(directory + "/session_details.txt");
  std::string token;
  int prev_session_id, prev_session_vertices, prev_session_edges;
  while (!ifs.eof()) {
    ifs >> token;
    if (token == "session_id") {
      ifs >> prev_session_id;
      current_session_id = prev_session_id + 1;
    } else if (token == "session_graph_vertices") {
      ifs >> prev_session_vertices;
    } else if (token == "session_graph_edges") {
      ifs >> prev_session_edges;
    }
  }

  // update the covisibility graph vertices and edges
  covisibility_graph->set_total_nbr_of_vertices(prev_session_vertices);
  covisibility_graph->set_total_nbr_of_edges(prev_session_edges);

  boost::filesystem::path floor_parent_path(directory + "/floors");
  if (boost::filesystem::is_directory(floor_parent_path)) {
    for (const auto& entry : boost::filesystem::directory_iterator(floor_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        floor_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(floor_directories);

  for (long unsigned int i = 0; i < floor_directories.size(); i++) {
    Floors floor;
    floor.load(floor_directories[i], covisibility_graph);
    floors_vec.insert({floor.id, floor});
  }

  boost::filesystem::path keyframe_parent_path(directory + "/keyframes");
  if (boost::filesystem::is_directory(keyframe_parent_path)) {
    for (const auto& entry :
         boost::filesystem::directory_iterator(keyframe_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        keyframe_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(keyframe_directories);

  for (long unsigned int i = 0; i < keyframe_directories.size(); i++) {
    auto keyframe =
        std::make_shared<KeyFrame>(keyframe_directories[i], covisibility_graph);
    keyframe->session_id = prev_session_id;
    loaded_keyframes.insert({keyframe->id(), keyframe});
  }

  for (auto it = loaded_keyframes.begin(); it != std::prev(loaded_keyframes.end());
       ++it) {
    KeyFrame::Ptr& prev_keyframe = it->second;
    KeyFrame::Ptr& keyframe = std::next(it)->second;
    keyframe_mapper->map_saved_keyframes(covisibility_graph, keyframe, prev_keyframe);
  }

  for (const auto& loaded_kf : loaded_keyframes) keyframes.insert(loaded_kf);
  for (const auto& kf : keyframes) std::cout << "kf id: " << kf.first << std::endl;

  boost::filesystem::path x_vert_plane_parent_path(directory + "/x_vert_planes");
  if (boost::filesystem::is_directory(x_vert_plane_parent_path)) {
    for (const auto& entry :
         boost::filesystem::directory_iterator(x_vert_plane_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        x_planes_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(x_planes_directories);

  for (long unsigned int i = 0; i < x_planes_directories.size(); i++) {
    VerticalPlanes vert_plane;
    vert_plane.load(x_planes_directories[i], covisibility_graph, "x");
    x_vert_planes.insert({vert_plane.id, vert_plane});
  }

  for (const auto& x_vert_plane : x_vert_planes) {
    plane_mapper->factor_saved_planes<VerticalPlanes>(covisibility_graph,
                                                      x_vert_plane.second);
    if (x_vert_plane.second.duplicate_id != -1)
      plane_mapper->factor_saved_duplicate_planes<VerticalPlanes>(
          covisibility_graph, x_vert_planes, x_vert_plane.second);
  }

  boost::filesystem::path y_vert_plane_parent_path(directory + "/y_vert_planes");
  if (boost::filesystem::is_directory(y_vert_plane_parent_path)) {
    for (const auto& entry :
         boost::filesystem::directory_iterator(y_vert_plane_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        y_planes_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(y_planes_directories);

  for (long unsigned int i = 0; i < y_planes_directories.size(); i++) {
    VerticalPlanes vert_plane;
    vert_plane.load(y_planes_directories[i], covisibility_graph, "y");
    y_vert_planes.insert({vert_plane.id, vert_plane});
  }

  for (const auto& y_vert_plane : y_vert_planes) {
    plane_mapper->factor_saved_planes<VerticalPlanes>(covisibility_graph,
                                                      y_vert_plane.second);
    if (y_vert_plane.second.duplicate_id != -1)
      plane_mapper->factor_saved_duplicate_planes<VerticalPlanes>(
          covisibility_graph, y_vert_planes, y_vert_plane.second);
  }

  boost::filesystem::path hort_plane_parent_path(directory + "/hort_planes");
  if (boost::filesystem::is_directory(hort_plane_parent_path)) {
    for (const auto& entry :
         boost::filesystem::directory_iterator(hort_plane_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        hort_plane_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(hort_plane_directories);

  for (long unsigned int i = 0; i < hort_plane_directories.size(); i++) {
    HorizontalPlanes hort_plane;
    hort_plane.load(hort_plane_directories[i], covisibility_graph, "h");
    hort_planes.insert({hort_plane.id, hort_plane});
  }

  for (const auto& hort_plane : hort_planes) {
    plane_mapper->factor_saved_planes<HorizontalPlanes>(covisibility_graph,
                                                        hort_plane.second);
    if (hort_plane.second.duplicate_id != -1)
      plane_mapper->factor_saved_duplicate_planes<HorizontalPlanes>(
          covisibility_graph, hort_planes, hort_plane.second);
  }

  boost::filesystem::path wall_parent_path(directory + "/walls");
  if (boost::filesystem::is_directory(wall_parent_path)) {
    for (const auto& entry : boost::filesystem::directory_iterator(wall_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        wall_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(wall_directories);

  for (long unsigned int i = 0; i < wall_directories.size(); i++) {
    Walls wall;
    wall.load(wall_directories[i], covisibility_graph);
    walls_vec.insert({wall.id, wall});
  }

  for (auto& wall : walls_vec) {
    if (wall.second.plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      wall_mapper->add_saved_walls(covisibility_graph, x_vert_planes, wall.second);
    } else {
      wall_mapper->add_saved_walls(covisibility_graph, y_vert_planes, wall.second);
    }
  }

  boost::filesystem::path room_parent_path(directory + "/rooms");
  if (boost::filesystem::is_directory(room_parent_path)) {
    for (const auto& entry : boost::filesystem::directory_iterator(room_parent_path)) {
      if (boost::filesystem::is_directory(entry.path())) {
        room_directories.push_back(entry.path().string());
      }
    }
  }
  sort_directories(room_directories);

  for (long unsigned int i = 0; i < room_directories.size(); i++) {
    Rooms room;
    std::vector<int> rooms_kf_ids;
    room.load(room_directories[i], rooms_kf_ids, covisibility_graph);
    for (const auto& room_kf_id : rooms_kf_ids) {
      auto kf = keyframes.find(room_kf_id);
      room.room_keyframes.insert({kf->first, kf->second});
    }
    rooms_vec.insert({room.id, room});
  }

  for (auto& room : rooms_vec) {
    finite_room_mapper->factor_saved_rooms(
        covisibility_graph, x_vert_planes, y_vert_planes, room.second);
  }

  // add anchor node to the first kf
  if (anchor_node == nullptr && this->get_parameter("fix_first_node_adaptive")
                                    .get_parameter_value()
                                    .get<bool>()) {
    keyframe_mapper->add_anchor_node(
        covisibility_graph, keyframes.begin()->second, anchor_node, anchor_edge);
  }

  current_floor_level = floors_vec.begin()->first;
  res->success = true;
  return true;
}

void SGraphsNode::sort_directories(std::vector<std::string>& directories) {
  std::sort(directories.begin(),
            directories.end(),
            [](const std::string& a, const std::string& b) {
              // Extract the last part of the path
              std::string a_last_part = boost::filesystem::path(a).filename().string();
              std::string b_last_part = boost::filesystem::path(b).filename().string();
              try {
                return std::stoi(a_last_part) < std::stoi(b_last_part);
              } catch (const std::invalid_argument&) {
                // Handle invalid conversion
                std::cerr << "Invalid directory name: " << a_last_part << " or "
                          << b_last_part << std::endl;
                return a_last_part < b_last_part;
              }
            });
}
}  // namespace s_graphs