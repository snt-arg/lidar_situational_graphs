// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <s_graphs/FloorCoeffs.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>

#include <s_graphs/SaveMap.h>
#include <s_graphs/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <s_graphs/ros_utils.hpp>
#include <s_graphs/ros_time_hash.hpp>
#include <s_graphs/PointClouds.h>
#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>

#include <s_graphs/graph_slam.hpp>
#include <s_graphs/keyframe.hpp>
#include <s_graphs/planes.hpp>
#include <s_graphs/corridors.hpp>
#include <s_graphs/rooms.hpp>
#include <s_graphs/keyframe_updater.hpp>
#include <s_graphs/loop_detector.hpp>
#include <s_graphs/information_matrix_calculator.hpp>
#include <s_graphs/map_cloud_generator.hpp>
#include <s_graphs/nmea_sentence_parser.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/room_mapper.hpp>
#include <s_graphs/plane_mapper.hpp>
#include <s_graphs/neighbour_mapper.hpp>
#include <s_graphs/plane_analyzer.hpp>

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

class SGraphsNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZRGBNormal PointNormal;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ApproxSyncPolicy;

  SGraphsNodelet() {}
  virtual ~SGraphsNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    wait_trans_odom2map = private_nh.param<bool>("wait_trans_odom2map", false);
    got_trans_odom2map = false;
    trans_odom2map.setIdentity();
    odom_path_vec.clear();
    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);
    floor_center_data.id = -1;

    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    plane_analyzer.reset(new PlaneAnalyzer(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());
    plane_utils.reset(new PlaneUtils());
    plane_mapper.reset(new PlaneMapper(private_nh));
    inf_room_mapper.reset(new InfiniteRoomMapper(private_nh));
    finite_room_mapper.reset(new FiniteRoomMapper(private_nh));
    neighbour_mapper.reset(new NeighbourMapper(private_nh));

    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);

    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

    keyframe_window_size = private_nh.param<int>("keyframe_window_size", 1);
    extract_planar_surfaces = private_nh.param<bool>("extract_planar_surfaces", true);
    plane_dist_threshold = private_nh.param<double>("plane_dist_threshold", 0.15);
    plane_points_dist = private_nh.param<double>("plane_points_dist", 0.5);
    constant_covariance = private_nh.param<bool>("constant_covariance", true);
    min_plane_points = private_nh.param<double>("min_plane_points", 100);
    use_point_to_plane = private_nh.param<bool>("use_point_to_plane", false);
    use_parallel_plane_constraint = private_nh.param<bool>("use_parallel_plane_constraint", false);
    use_perpendicular_plane_constraint = private_nh.param<bool>("use_perpendicular_plane_constraint", false);

    use_corridor_constraint = private_nh.param<bool>("use_corridor_constraint", false);
    corridor_information = private_nh.param<double>("corridor_information", 0.01);
    corridor_dist_threshold = private_nh.param<double>("corridor_dist_threshold", 1.0);
    corridor_min_plane_length = private_nh.param<double>("corridor_min_plane_length", 10);
    corridor_min_width = private_nh.param<double>("corridor_min_width", 1.5);
    corridor_max_width = private_nh.param<double>("corridor_max_width", 2.5);
    corridor_plane_length_diff_threshold = private_nh.param<double>("corridor_plane_length_diff_threshold", 0.3);
    corridor_point_diff_threshold = private_nh.param<double>("corridor_point_diff_threshold", 3.0);
    corridor_min_seg_dist = private_nh.param<double>("corridor_min_seg_dist", 1.5);

    use_room_constraint = private_nh.param<bool>("use_room_constraint", false);
    room_information = private_nh.param<double>("room_information", 0.01);
    room_plane_length_diff_threshold = private_nh.param<double>("room_plane_length_diff_threshold", 0.3);
    room_dist_threshold = private_nh.param<double>("room_dist_threshold", 1.0);
    room_min_plane_length = private_nh.param<double>("room_min_plane_length", 3.0);
    room_max_plane_length = private_nh.param<double>("room_max_plane_length", 6.0);
    room_min_width = private_nh.param<double>("room_min_width", 2.5);
    room_max_width = private_nh.param<double>("room_max_width", 6.0);
    room_point_diff_threshold = private_nh.param<double>("room_point_diff_threshold", 3.0);
    room_width_diff_threshold = private_nh.param<double>("room_width_diff_threshold", 2.5);

    color_r = private_nh.param<double>("color_r", 1);
    color_g = private_nh.param<double>("color_g", 1);
    color_b = private_nh.param<double>("color_b", 1);

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

    init_odom2map_sub = nh.subscribe("/odom2map/initial_pose", 1, &SGraphsNodelet::init_map2odom_pose_callback, this);
    while(wait_trans_odom2map && !got_trans_odom2map) {
      ROS_WARN("Waiting for the Initial Transform between odom and map frame");
      ros::spinOnce();
      usleep(1e6);
    }

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&SGraphsNodelet::cloud_callback, this, _1, _2));

    raw_odom_sub = nh.subscribe("/odom", 1, &SGraphsNodelet::raw_odom_callback, this);
    imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &SGraphsNodelet::imu_callback, this);
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &SGraphsNodelet::floor_coeffs_callback, this);
    room_data_sub = nh.subscribe("/room_segmentation/room_data", 1, &SGraphsNodelet::room_data_callback, this);
    all_room_data_sub = nh.subscribe("/floor_plan/all_rooms_data", 1, &SGraphsNodelet::all_room_data_callback, this);
    floor_data_sub = nh.subscribe("/floor_plan/floor_data", 1, &SGraphsNodelet::floor_data_callback, this);

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &SGraphsNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &SGraphsNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &SGraphsNodelet::navsat_callback, this);
    }

    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/s_graphs/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/s_graphs/odom2map", 16);
    odom_pose_corrected_pub = mt_nh.advertise<geometry_msgs::PoseStamped>("/s_graphs/odom_pose_corrected", 10);
    odom_path_corrected_pub = mt_nh.advertise<nav_msgs::Path>("/s_graphs/odom_path_corrected", 10);

    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/s_graphs/map_points", 1, true);
    map_planes_pub = mt_nh.advertise<s_graphs::PlanesData>("/s_graphs/map_planes", 1, false);
    all_map_planes_pub = mt_nh.advertise<s_graphs::PlanesData>("/s_graphs/all_map_planes", 1, false);

    read_until_pub = mt_nh.advertise<std_msgs::Header>("/s_graphs/read_until", 32);
    dump_service_server = mt_nh.advertiseService("/s_graphs/dump", &SGraphsNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/s_graphs/save_map", &SGraphsNodelet::save_map_service, this);

    graph_updated = false;
    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createTimer(ros::Duration(graph_update_interval), &SGraphsNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createTimer(ros::Duration(map_cloud_update_interval), &SGraphsNodelet::map_points_publish_timer_callback, this);
  }

private:
  /**
   * @brief receive the raw odom msg to publish the corrected odom after s
   *
   */
  void raw_odom_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
    Eigen::Isometry3d odom = odom2isometry(odom_msg);
    trans_odom2map_mutex.lock();
    Eigen::Matrix4f odom_corrected = trans_odom2map * odom.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    geometry_msgs::PoseStamped pose_stamped_corrected = matrix2PoseStamped(odom_msg->header.stamp, odom_corrected, map_frame_id);
    publish_corrected_odom(pose_stamped_corrected);
  }

  /**
   * @brief receive the initial transform between map and odom frame
   * @param map2odom_pose_msg
   */
  void init_map2odom_pose_callback(const geometry_msgs::PoseStamped pose_msg) {
    if(got_trans_odom2map) return;

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z).toRotationMatrix();

    trans_odom2map.block<3, 3>(0, 0) = mat3;
    trans_odom2map(0, 3) = pose_msg.pose.position.x;
    trans_odom2map(1, 3) = pose_msg.pose.position.y;
    trans_odom2map(2, 3) = pose_msg.pose.position.z;

    if(trans_odom2map.isIdentity())
      return;
    else {
      got_trans_odom2map = true;
    }
  }

  /**
   * @brief get the room data from room segmentation module
   *
   */
  void room_data_callback(const s_graphs::RoomsData rooms_msg) {
    std::lock_guard<std::mutex> lock(room_data_queue_mutex);
    room_data_queue.push_back(rooms_msg);
    // std::cout << "pre_room_data_vec size :" << pre_room_data_vec.size() << std::endl;
  }

  void floor_data_callback(const s_graphs::RoomData floor_data_msg) {
    std::lock_guard<std::mutex> lock(floor_data_mutex);
    floor_center_data = floor_data_msg;
  }

  /**
   * @brief flush the room data from room data queue
   *
   */
  void flush_room_data_queue() {
    std::lock_guard<std::mutex> lock(room_data_queue_mutex);

    if(keyframes.empty()) {
      return;
    } else if(room_data_queue.empty()) {
      std::cout << "room data queue is empty" << std::endl;
      return;
    }

    for(const auto& room_data_msg : room_data_queue) {
      for(const auto& room_data : room_data_msg.rooms) {
        // float dist_robot_room = sqrt(pow(room_data.room_center.x - latest_keyframe->node->estimate().matrix()(0,3),2) + pow(room_data.room_center.y - latest_keyframe->node->estimate().matrix()(1,3),2));
        // std::cout << "dist robot room: " << dist_robot_room << std::endl;
        if(room_data.x_planes.size() == 2 && room_data.y_planes.size() == 2) {
          finite_room_mapper->lookup_rooms(graph_slam, room_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors, rooms_vec);
        }
        // x corridor
        else if(room_data.x_planes.size() == 2 && room_data.y_planes.size() == 0) {
          inf_room_mapper->lookup_corridors(graph_slam, PlaneUtils::plane_class::X_VERT_PLANE, room_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors, rooms_vec);
        }
        // y corridor
        else if(room_data.x_planes.size() == 0 && room_data.y_planes.size() == 2) {
          inf_room_mapper->lookup_corridors(graph_slam, PlaneUtils::plane_class::Y_VERT_PLANE, room_data, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors, rooms_vec);
        }
      }

      room_data_queue.pop_front();
    }
  }

  /**
   * @brief get the entire room data from floor plan module to detect neighbours
   *
   */
  void all_room_data_callback(const s_graphs::RoomsData rooms_msg) {
    std::lock_guard<std::mutex> lock(all_room_data_queue_mutex);
    all_room_data_queue.push_back(rooms_msg);
  }

  void flush_all_room_data_queue() {
    std::lock_guard<std::mutex> lock(all_room_data_queue_mutex);

    if(keyframes.empty()) {
      return;
    } else if(all_room_data_queue.empty()) {
      std::cout << "all room data queue is empty" << std::endl;
      return;
    }

    for(const auto& room_data_msg : all_room_data_queue) {
      neighbour_mapper->detect_room_neighbours(graph_slam, room_data_msg, x_corridors, y_corridors, rooms_vec);
      neighbour_mapper->factor_room_neighbours(graph_slam, room_data_msg, x_corridors, y_corridors, rooms_vec);

      all_room_data_queue.pop_front();
    }
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const ros::Time& stamp = cloud_msg->header.stamp;
    Eigen::Isometry3d odom = odom2isometry(odom_msg);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }

      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      std::cout << "keyframe_queue is empty " << std::endl;
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(private_nh.param<bool>("fix_first_node", false)) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::stringstream sst(private_nh.param<std::string>("fix_first_node_stddev", "1 1 1 1 1 1"));
          for(int i = 0; i < 6; i++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(i, i) = 1.0 / stddev;
          }

          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      // perform planar segmentation
      if(extract_planar_surfaces) {
        std::vector<sensor_msgs::PointCloud2> extracted_cloud_vec = plane_analyzer->get_segmented_planes(keyframe->cloud);
        map_extracted_planes(keyframe, extracted_cloud_vec);
      }

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  void map_extracted_planes(KeyFrame::Ptr keyframe, const std::vector<sensor_msgs::PointCloud2>& extracted_cloud_vec) {
    std::vector<plane_data_list> x_det_corridor_candidates, y_det_corridor_candidates;
    std::vector<plane_data_list> x_det_room_candidates, y_det_room_candidates;

    for(const auto& cloud_seg_msg : extracted_cloud_vec) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_body(new pcl::PointCloud<PointNormal>());
      pcl::fromROSMsg(cloud_seg_msg, *cloud_seg_body);

      if(cloud_seg_body->points.size() < min_plane_points) continue;
      keyframe->cloud_seg_body = cloud_seg_body;

      g2o::Plane3D det_plane_body_frame = Eigen::Vector4d(cloud_seg_body->back().normal_x, cloud_seg_body->back().normal_y, cloud_seg_body->back().normal_z, cloud_seg_body->back().curvature);
      bool found_corridor_candidates = false;
      bool found_room_candidates = false;
      plane_data_list plane_id_pair;
      int plane_type = plane_mapper->map_detected_planes(graph_slam, keyframe, det_plane_body_frame, found_corridor_candidates, found_room_candidates, plane_id_pair, x_vert_planes, y_vert_planes, hort_planes);
      switch(plane_type) {
        case PlaneUtils::plane_class::X_VERT_PLANE: {
          if(found_corridor_candidates) {
            x_det_corridor_candidates.push_back(plane_id_pair);
          }
          if(found_room_candidates) {
            x_det_room_candidates.push_back(plane_id_pair);
          }
          break;
        }
        case PlaneUtils::plane_class::Y_VERT_PLANE: {
          if(found_corridor_candidates) {
            y_det_corridor_candidates.push_back(plane_id_pair);
          }
          if(found_room_candidates) {
            y_det_room_candidates.push_back(plane_id_pair);
          }
          break;
        }
        case PlaneUtils::plane_class::HORT_PLANE: {
          break;
        }
        default: {
          break;
        }
      }
    }

    if(use_corridor_constraint) {
      inf_room_mapper->lookup_corridors(graph_slam, x_det_corridor_candidates, y_det_corridor_candidates, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, x_corridors, y_corridors);
    }

    if(use_room_constraint) {
      finite_room_mapper->lookup_rooms(graph_slam, x_det_room_candidates, y_det_room_candidates, x_vert_planes, y_vert_planes, dupl_x_vert_planes, dupl_y_vert_planes, rooms_vec);
    }
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
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
  void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if(0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

      // the first gps data position will be the origin of the map
      if(!zero_utm) {
        zero_utm = xyz;
      }
      xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(xyz.z())) {
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) { return stamp < geopoint->header.stamp; });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
    if(!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp += ros::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::Vector3Stamped acc_imu;
      geometry_msgs::Vector3Stamped acc_base;
      geometry_msgs::QuaternionStamped quat_imu;
      geometry_msgs::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;

      try {
        tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
        tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
      } catch(std::exception& e) {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }

      keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
      keyframe->orientation = keyframe->orientation;
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if(enable_imu_orientation) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
      }

      if(enable_imu_acceleration) {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) { return stamp < imu->header.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const s_graphs::FloorCoeffsConstPtr& floor_coeffs_msg) {
    if(floor_coeffs_msg->coeffs.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs->header.stamp > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(floor_coeffs->header.stamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      if(!floor_plane_node) {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const ros::Time& stamp, const s_graphs::FloorCoeffsConstPtr& coeffs) { return stamp < coeffs->header.stamp; });
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback(const ros::TimerEvent& event) {
    if(!map_points_pub.getNumSubscribers() || !graph_updated) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
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

    std::vector<Corridors> x_corridor_snapshot, y_corridor_snapshot;
    corridor_snapshot_mutex.lock();
    x_corridor_snapshot = x_corridors_snapshot;
    y_corridor_snapshot = y_corridors_snapshot;
    corridor_snapshot_mutex.unlock();

    std::vector<Rooms> room_snapshot;
    room_snapshot_mutex.lock();
    room_snapshot = rooms_vec_snapshot;
    room_snapshot_mutex.unlock();

    auto markers = create_marker_array(ros::Time::now(), local_graph, x_plane_snapshot, y_plane_snapshot, hort_plane_snapshot, x_corridor_snapshot, y_corridor_snapshot, room_snapshot);
    markers_pub.publish(markers);

    publish_all_mapped_planes(x_plane_snapshot, y_plane_snapshot);
    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::Header read_until;
      read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);
    }

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // publish mapped planes
    publish_mapped_planes(x_vert_planes, y_vert_planes);

    // flush the room poses from room detector and no need to return if no rooms found
    flush_room_data_queue();

    // flush all the rooms queue to map neighbours
    // flush_all_room_data_queue();

    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    if(anchor_node && private_nh.param<bool>("fix_first_node_adaptive", true)) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    // optimize the pose graph
    int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);

    if((graph_slam->optimize(num_iterations)) > 0 && !constant_covariance) compute_plane_cov();

    // merge_duplicate_planes();

    vert_plane_snapshot_mutex.lock();
    x_vert_planes_snapshot = x_vert_planes;
    y_vert_planes_snapshot = y_vert_planes;
    vert_plane_snapshot_mutex.unlock();

    hort_plane_snapshot_mutex.lock();
    hort_planes_snapshot = hort_planes;
    hort_plane_snapshot_mutex.unlock();

    corridor_snapshot_mutex.lock();
    x_corridors_snapshot = x_corridors;
    y_corridors_snapshot = y_corridors;
    corridor_snapshot_mutex.unlock();

    room_snapshot_mutex.lock();
    rooms_vec_snapshot = rooms_vec;
    room_snapshot_mutex.unlock();

    graph_mutex.lock();
    graph_snapshot = graph_slam->graph.get();
    graph_mutex.unlock();

    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
    odom2map_pub.publish(ts);
  }

  /**
   * @brief publish the mapped plane information from the last n keyframes
   *
   */
  void publish_mapped_planes(std::vector<VerticalPlanes> x_vert_planes_snapshot, std::vector<VerticalPlanes> y_vert_planes_snapshot) {
    if(keyframes.empty()) return;

    std::vector<KeyFrame::Ptr> keyframe_window(keyframes.end() - std::min<int>(keyframes.size(), keyframe_window_size), keyframes.end());
    std::map<int, int> unique_x_plane_ids, unique_y_plane_ids;
    for(std::vector<KeyFrame::Ptr>::reverse_iterator it = keyframe_window.rbegin(); it != keyframe_window.rend(); ++it) {
      for(const auto& x_plane_id : (*it)->x_plane_ids) {
        auto result = unique_x_plane_ids.insert(std::pair<int, int>(x_plane_id, 1));
        // if(result.second == false) std::cout << "x plane already existed with id : " << x_plane_id << std::endl;
      }

      for(const auto& y_plane_id : (*it)->y_plane_ids) {
        auto result = unique_y_plane_ids.insert(std::pair<int, int>(y_plane_id, 1));
        // if(result.second == false) std::cout << "y plane already existed with id : " << y_plane_id << std::endl;
      }
    }

    s_graphs::PlanesData vert_planes_data;
    vert_planes_data.header.stamp = keyframes.back()->stamp;
    for(const auto& unique_x_plane_id : unique_x_plane_ids) {
      auto local_x_vert_plane = std::find_if(x_vert_planes_snapshot.begin(), x_vert_planes_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == unique_x_plane_id.first);
      if(local_x_vert_plane == x_vert_planes_snapshot.end()) continue;
      s_graphs::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (*local_x_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, mapped_plane_coeffs);
      plane_data.id = (*local_x_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for(const auto& plane_point_data : (*local_x_vert_plane).cloud_seg_map->points) {
        geometry_msgs::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.x_planes.push_back(plane_data);
    }

    for(const auto& unique_y_plane_id : unique_y_plane_ids) {
      auto local_y_vert_plane = std::find_if(y_vert_planes_snapshot.begin(), y_vert_planes_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == unique_y_plane_id.first);
      if(local_y_vert_plane == y_vert_planes_snapshot.end()) continue;
      s_graphs::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (*local_y_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, mapped_plane_coeffs);
      plane_data.id = (*local_y_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for(const auto& plane_point_data : (*local_y_vert_plane).cloud_seg_map->points) {
        geometry_msgs::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.y_planes.push_back(plane_data);
    }
    map_planes_pub.publish(vert_planes_data);
  }

  /**
   * @brief publish all the mapped plane information from the entire set of keyframes
   *
   */
  void publish_all_mapped_planes(const std::vector<VerticalPlanes>& x_vert_planes_snapshot, const std::vector<VerticalPlanes>& y_vert_planes_snapshot) {
    if(keyframes.empty()) return;

    s_graphs::PlanesData vert_planes_data;
    vert_planes_data.header.stamp = keyframes.back()->stamp;
    for(const auto& x_vert_plane : x_vert_planes_snapshot) {
      s_graphs::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (x_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, mapped_plane_coeffs);
      plane_data.id = (x_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for(const auto& plane_point_data : (x_vert_plane).cloud_seg_map->points) {
        geometry_msgs::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.x_planes.push_back(plane_data);
    }

    for(const auto& y_vert_plane : y_vert_planes_snapshot) {
      s_graphs::PlaneData plane_data;
      Eigen::Vector4d mapped_plane_coeffs;
      mapped_plane_coeffs = (y_vert_plane).plane_node->estimate().coeffs();
      // correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, mapped_plane_coeffs);
      plane_data.id = (y_vert_plane).id;
      plane_data.nx = mapped_plane_coeffs(0);
      plane_data.ny = mapped_plane_coeffs(1);
      plane_data.nz = mapped_plane_coeffs(2);
      plane_data.d = mapped_plane_coeffs(3);
      for(const auto& plane_point_data : (y_vert_plane).cloud_seg_map->points) {
        geometry_msgs::Vector3 plane_point;
        plane_point.x = plane_point_data.x;
        plane_point.y = plane_point_data.y;
        plane_point.z = plane_point_data.z;
        plane_data.plane_points.push_back(plane_point);
      }
      vert_planes_data.y_planes.push_back(plane_data);
    }
    all_map_planes_pub.publish(vert_planes_data);
  }

  /**
   * @brief merge all the duplicate x and y planes detected by room/corridors
   */
  void merge_duplicate_planes() {
    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> curr_dupl_x_vert_planes;
    // check the number of occurances of the same duplicate planes
    for(auto it_1 = dupl_x_vert_planes.begin(); it_1 != dupl_x_vert_planes.end(); ++it_1) {
      int id_count = 0;
      int current_id = (*it_1).second.id;
      for(auto it_2 = dupl_x_vert_planes.begin(); it_2 != dupl_x_vert_planes.end(); ++it_2) {
        if(current_id == (*it_2).second.id) {
          id_count++;
        }
      }
      if(id_count > 3) {
        curr_dupl_x_vert_planes.push_back(*it_1);
      }
    }

    for(auto it = curr_dupl_x_vert_planes.begin(); it != curr_dupl_x_vert_planes.end(); ++it) {
      std::set<g2o::HyperGraph::Edge*> edges = (*it).first.plane_node->edges();

      for(auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(*edge_itr);
        if(edge_se3_plane) {
          /* get the keyframe node and connect it with the original mapped plane node */
          g2o::VertexSE3* keyframe_node = dynamic_cast<g2o::VertexSE3*>(edge_se3_plane->vertices()[0]);
          g2o::Plane3D local_plane = keyframe_node->estimate().inverse() * (*it).second.plane_node->estimate();
          Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
          auto edge = graph_slam->add_se3_plane_edge(keyframe_node, (*it).second.plane_node, local_plane.coeffs(), information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);

          /* remove the edge between the keyframe and found duplicate plane */
          if(graph_slam->remove_se3_plane_edge(edge_se3_plane)) std::cout << "removed edge - pose se3 x plane " << std::endl;
          continue;
        }

        g2o::EdgeCorridorXPlane* edge_corridor_xplane = dynamic_cast<g2o::EdgeCorridorXPlane*>(*edge_itr);
        if(edge_corridor_xplane) {
          /* remove the edge between the corridor and the duplicate found plane */
          /* get corridor id from the vertex */
          g2o::VertexCorridor* corridor_node = dynamic_cast<g2o::VertexCorridor*>(edge_corridor_xplane->vertices()[0]);
          auto found_x_corridor = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::id, _1) == corridor_node->id());
          /* if any of the mapped plane_id of the corridor equal to dupl plane id replace it */
          if((*found_x_corridor).plane1_id == (*it).first.id) {
            (*found_x_corridor).plane1_id = (*it).second.id;
            (*found_x_corridor).plane1 = (*it).second.plane;
          } else if((*found_x_corridor).plane2_id == (*it).first.id) {
            (*found_x_corridor).plane2_id = (*it).second.id;
            (*found_x_corridor).plane2 = (*it).second.plane;
          }
          /* Add edge between corridor and current mapped plane */
          Eigen::Vector4d found_mapped_plane1_coeffs = (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_plane1_coeffs, (*it).second.cloud_seg_map->points.back().x, (*it).second.cloud_seg_map->points.back().y);
          double meas_plane1 = inf_room_mapper->corridor_measurement(PlaneUtils::plane_class::X_VERT_PLANE, corridor_node->estimate(), found_mapped_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_corridor_plane(corridor_information);
          auto edge_plane = graph_slam->add_corridor_xplane_edge(corridor_node, (*it).second.plane_node, meas_plane1, information_corridor_plane);
          graph_slam->add_robust_kernel(edge_plane, "Huber", 1.0);

          if(graph_slam->remove_corridor_xplane_edge(edge_corridor_xplane)) std::cout << "removed edge - corridor xplane " << std::endl;
          continue;
        }
        /* TODO: analyze if connecting room node with (*it).second.plane is necessary  */
        g2o::EdgeRoomXPlane* edge_room_xplane = dynamic_cast<g2o::EdgeRoomXPlane*>(*edge_itr);
        if(edge_room_xplane) {
          /* remove the edge between the room and the duplicate found plane */
          /* get room id from the vertex */
          g2o::VertexRoomXYLB* room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_xplane->vertices()[0]);
          auto found_room = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::id, _1) == room_node->id());
          if((*found_room).plane_x1_id == (*it).first.id) {
            (*found_room).plane_x1_id = (*it).second.id;
            (*found_room).plane_x1 = (*it).second.plane;
          } else if((*found_room).plane_x2_id == (*it).first.id) {
            (*found_room).plane_x2_id = (*it).second.id;
            (*found_room).plane_x2 = (*it).second.plane;
          }

          /* Add edge between room and current mapped plane */
          Eigen::Vector4d found_mapped_x_plane1_coeffs = (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, found_mapped_x_plane1_coeffs, (*it).second.cloud_seg_map->points.back().x, (*it).second.cloud_seg_map->points.back().y);
          Eigen::Vector2d x_plane1_meas = finite_room_mapper->room_measurement(PlaneUtils::plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane1_coeffs);
          Eigen::Matrix2d information_room_plane;
          information_room_plane(0, 0) = room_information;
          information_room_plane(1, 1) = room_information;
          auto edge_x_plane1 = graph_slam->add_room_xplane_edge(room_node, (*it).second.plane_node, x_plane1_meas, information_room_plane);
          graph_slam->add_robust_kernel(edge_x_plane1, "Huber", 1.0);

          if(graph_slam->remove_room_xplane_edge(edge_room_xplane)) std::cout << "removed edge - room xplane " << std::endl;
          continue;
        }
      }
      /* finally remove the duplicate plane node */
      if(graph_slam->remove_plane_node((*it).first.plane_node)) {
        auto mapped_plane = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == (*it).first.id);
        x_vert_planes.erase(mapped_plane);
        std::cout << "removed x vert plane " << std::endl;
      }
    }

    // remove only the current detected duplicate planes
    for(int i = 0; i < curr_dupl_x_vert_planes.size(); ++i) {
      for(int j = 0; j < dupl_x_vert_planes.size();) {
        if(curr_dupl_x_vert_planes[i].second.id == dupl_x_vert_planes[j].second.id) {
          dupl_x_vert_planes.erase(dupl_x_vert_planes.begin() + j);
        } else
          ++j;
      }
    }

    std::deque<std::pair<VerticalPlanes, VerticalPlanes>> curr_dupl_y_vert_planes;
    // check the number of occurances of the same duplicate planes
    for(auto it_1 = dupl_y_vert_planes.begin(); it_1 != dupl_y_vert_planes.end(); ++it_1) {
      int id_count = 0;
      int current_id = (*it_1).second.id;
      for(auto it_2 = dupl_y_vert_planes.begin(); it_2 != dupl_y_vert_planes.end(); ++it_2) {
        if(current_id == (*it_2).second.id) {
          id_count++;
        }
      }
      if(id_count > 3) {
        curr_dupl_y_vert_planes.push_back(*it_1);
      }
    }

    for(auto it = curr_dupl_y_vert_planes.begin(); it != curr_dupl_y_vert_planes.end(); ++it) {
      std::set<g2o::HyperGraph::Edge*> edges = (*it).first.plane_node->edges();

      for(auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(*edge_itr);
        if(edge_se3_plane) {
          /* get the keyframe node and connect it with the original mapped plane node */
          g2o::VertexSE3* keyframe_node = dynamic_cast<g2o::VertexSE3*>(edge_se3_plane->vertices()[0]);
          g2o::Plane3D local_plane = keyframe_node->estimate().inverse() * (*it).second.plane_node->estimate();
          Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
          auto edge = graph_slam->add_se3_plane_edge(keyframe_node, (*it).second.plane_node, local_plane.coeffs(), information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);

          /* remove the edge between the keyframe and found duplicate plane */
          if(graph_slam->remove_se3_plane_edge(edge_se3_plane)) std::cout << "remove edge - pose se3 yplane " << std::endl;
          continue;
        }
        g2o::EdgeCorridorYPlane* edge_corridor_yplane = dynamic_cast<g2o::EdgeCorridorYPlane*>(*edge_itr);
        if(edge_corridor_yplane) {
          /* remove the edge between the corridor and the duplicate found plane */
          g2o::VertexCorridor* corridor_node = dynamic_cast<g2o::VertexCorridor*>(edge_corridor_yplane->vertices()[0]);
          auto found_y_corridor = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::id, _1) == corridor_node->id());
          if((*found_y_corridor).plane1_id == (*it).first.id) {
            (*found_y_corridor).plane1_id = (*it).second.id;
            (*found_y_corridor).plane1 = (*it).second.plane;
          } else if((*found_y_corridor).plane2_id == (*it).first.id) {
            (*found_y_corridor).plane2_id = (*it).second.id;
            (*found_y_corridor).plane2 = (*it).second.plane;
          }

          /* Add edge between corridor and current mapped plane */
          Eigen::Vector4d found_mapped_plane1_coeffs = (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_plane1_coeffs, (*it).second.cloud_seg_map->points.back().x, (*it).second.cloud_seg_map->points.back().y);
          double meas_plane1 = inf_room_mapper->corridor_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, corridor_node->estimate(), found_mapped_plane1_coeffs);
          Eigen::Matrix<double, 1, 1> information_corridor_plane(corridor_information);
          auto edge_plane = graph_slam->add_corridor_yplane_edge(corridor_node, (*it).second.plane_node, meas_plane1, information_corridor_plane);
          graph_slam->add_robust_kernel(edge_plane, "Huber", 1.0);

          if(graph_slam->remove_corridor_yplane_edge(edge_corridor_yplane)) std::cout << "removed edge - corridor yplane " << std::endl;
          continue;
        }
        g2o::EdgeRoomYPlane* edge_room_yplane = dynamic_cast<g2o::EdgeRoomYPlane*>(*edge_itr);
        if(edge_room_yplane) {
          /* remove the edge between the room and the duplicate found plane */
          g2o::VertexRoomXYLB* room_node = dynamic_cast<g2o::VertexRoomXYLB*>(edge_room_yplane->vertices()[0]);
          auto found_room = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::id, _1) == room_node->id());
          if((*found_room).plane_y1_id == (*it).first.id) {
            (*found_room).plane_y1_id = (*it).second.id;
            (*found_room).plane_y1 = (*it).second.plane;
          } else if((*found_room).plane_y2_id == (*it).first.id) {
            (*found_room).plane_y2_id = (*it).second.id;
            (*found_room).plane_y2 = (*it).second.plane;
          }

          /* Add edge between room and current mapped plane */
          Eigen::Vector4d found_mapped_y_plane1_coeffs = (*it).second.plane_node->estimate().coeffs();
          plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, found_mapped_y_plane1_coeffs, (*it).second.cloud_seg_map->points.back().x, (*it).second.cloud_seg_map->points.back().y);
          Eigen::Vector2d y_plane1_meas = finite_room_mapper->room_measurement(PlaneUtils::plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane1_coeffs);
          Eigen::Matrix2d information_room_plane;
          information_room_plane(0, 0) = room_information;
          information_room_plane(1, 1) = room_information;
          auto edge_y_plane1 = graph_slam->add_room_xplane_edge(room_node, (*it).second.plane_node, y_plane1_meas, information_room_plane);
          graph_slam->add_robust_kernel(edge_y_plane1, "Huber", 1.0);

          if(graph_slam->remove_room_yplane_edge(edge_room_yplane)) std::cout << "removed edge - room yplane " << std::endl;
          continue;
        }
      }
      /* finally remove the duplicate plane node */
      if(graph_slam->remove_plane_node((*it).first.plane_node)) {
        auto mapped_plane = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == (*it).first.id);
        y_vert_planes.erase(mapped_plane);
        std::cout << "removed y vert plane " << std::endl;
      }
    }

    // remove only the current detected duplicate planes
    for(int i = 0; i < curr_dupl_y_vert_planes.size(); ++i) {
      for(int j = 0; j < dupl_y_vert_planes.size();) {
        if(curr_dupl_y_vert_planes[i].second.id == dupl_y_vert_planes[j].second.id) {
          dupl_y_vert_planes.erase(dupl_y_vert_planes.begin() + j);
        } else
          ++j;
      }
    }
  }

  /**
   * @brief publish odom corrected pose and path
   */
  void publish_corrected_odom(geometry_msgs::PoseStamped pose_stamped_corrected) {
    nav_msgs::Path path_stamped_corrected;
    path_stamped_corrected.header = pose_stamped_corrected.header;
    odom_path_vec.push_back(pose_stamped_corrected);
    path_stamped_corrected.poses = odom_path_vec;

    odom_pose_corrected_pub.publish(pose_stamped_corrected);
    odom_path_corrected_pub.publish(path_stamped_corrected);
  }

  /**
   * @brief compute the plane covariances
   */
  void compute_plane_cov() {
    g2o::SparseBlockMatrix<Eigen::MatrixXd> plane_spinv_vec;
    std::vector<std::pair<int, int>> plane_pairs_vec;
    for(int i = 0; i < x_vert_planes.size(); ++i) {
      x_vert_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(x_vert_planes[i].plane_node->hessianIndex(), x_vert_planes[i].plane_node->hessianIndex()));
    }
    for(int i = 0; i < y_vert_planes.size(); ++i) {
      y_vert_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(y_vert_planes[i].plane_node->hessianIndex(), y_vert_planes[i].plane_node->hessianIndex()));
    }
    for(int i = 0; i < hort_planes.size(); ++i) {
      hort_planes[i].plane_node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(hort_planes[i].plane_node->hessianIndex(), hort_planes[i].plane_node->hessianIndex()));
    }

    if(!plane_pairs_vec.empty()) {
      if(graph_slam->compute_landmark_marginals(plane_spinv_vec, plane_pairs_vec)) {
        int i = 0;
        while(i < x_vert_planes.size()) {
          // std::cout << "covariance of x plane " << i << " " << y_vert_planes[i].covariance << std::endl;
          x_vert_planes[i].covariance = plane_spinv_vec.block(x_vert_planes[i].plane_node->hessianIndex(), x_vert_planes[i].plane_node->hessianIndex())->eval().cast<double>();
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(x_vert_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of x plane not PSD" << i << " " << x_vert_planes[i].covariance << std::endl;
            x_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
        }
        i = 0;
        while(i < y_vert_planes.size()) {
          y_vert_planes[i].covariance = plane_spinv_vec.block(y_vert_planes[i].plane_node->hessianIndex(), y_vert_planes[i].plane_node->hessianIndex())->eval().cast<double>();
          // std::cout << "covariance of y plane " << i << " " << y_vert_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(y_vert_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of y plane not PSD " << i << " " << y_vert_planes[i].covariance << std::endl;
            y_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
        }
        i = 0;
        while(i < hort_planes.size()) {
          hort_planes[i].covariance = plane_spinv_vec.block(hort_planes[i].plane_node->hessianIndex(), hort_planes[i].plane_node->hessianIndex())->eval().cast<double>();
          // std::cout << "covariance of y plane " << i << " " << hort_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(hort_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
            // std::cout << "covariance of y plane not PSD " << i << " " << hort_planes[i].covariance << std::endl;
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
    for(int i = 0; i < x_vert_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<PointNormal>());

      for(int k = 0; k < x_vert_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose = x_vert_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();
        for(size_t j = 0; j < x_vert_planes[i].cloud_seg_body_vec[k]->points.size(); ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() = pose * x_vert_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      x_vert_planes[i].cloud_seg_map = cloud_seg_map;
    }

    for(int i = 0; i < y_vert_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<PointNormal>());

      for(int k = 0; k < y_vert_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose = y_vert_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();

        for(size_t j = 0; j < y_vert_planes[i].cloud_seg_body_vec[k]->points.size(); ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() = pose * y_vert_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      y_vert_planes[i].cloud_seg_map = cloud_seg_map;
    }

    for(int i = 0; i < hort_planes.size(); ++i) {
      pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<PointNormal>());

      for(int k = 0; k < hort_planes[i].keyframe_node_vec.size(); ++k) {
        Eigen::Matrix4f pose = hort_planes[i].keyframe_node_vec[k]->estimate().matrix().cast<float>();

        for(size_t j = 0; j < hort_planes[i].cloud_seg_body_vec[k]->points.size(); ++j) {
          PointNormal dst_pt;
          dst_pt.getVector4fMap() = pose * hort_planes[i].cloud_seg_body_vec[k]->points[j].getVector4fMap();
          cloud_seg_map->points.push_back(dst_pt);
        }
      }
      hort_planes[i].cloud_seg_map = cloud_seg_map;
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp, const g2o::SparseOptimizer* local_graph, const std::vector<VerticalPlanes>& x_plane_snapshot, const std::vector<VerticalPlanes>& y_plane_snapshot, const std::vector<HorizontalPlanes>& hort_plane_snapshot, std::vector<Corridors> x_corridor_snapshot, std::vector<Corridors> y_corridor_snapshot, const std::vector<Rooms>& room_snapshot) {
    visualization_msgs::MarkerArray markers;
    // markers.markers.resize(11);

    // node markers
    double keyframe_h = 5.0;
    double plane_h = 10;

    visualization_msgs::Marker traj_marker;
    traj_marker.header.frame_id = map_frame_id;
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = markers.markers.size();
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker imu_marker;
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = markers.markers.size() + 1;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i = 0; i < keyframes.size(); i++) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z() + keyframe_h;

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }
    markers.markers.push_back(traj_marker);
    markers.markers.push_back(imu_marker);

    // keyframe edge markers
    visualization_msgs::Marker traj_edge_marker;
    traj_edge_marker.header.frame_id = map_frame_id;
    traj_edge_marker.header.stamp = stamp;
    traj_edge_marker.ns = "keyframe_keyframe_edges";
    traj_edge_marker.id = markers.markers.size();
    traj_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    traj_edge_marker.pose.orientation.w = 1.0;
    traj_edge_marker.scale.x = 0.05;

    auto traj_edge_itr = local_graph->edges().begin();
    for(int i = 0; traj_edge_itr != local_graph->edges().end(); traj_edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *traj_edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        geometry_msgs::Point point1, point2;
        point1.x = pt1.x();
        point1.y = pt1.y();
        point1.z = pt1.z() + keyframe_h;

        point2.x = pt2.x();
        point2.y = pt2.y();
        point2.z = pt2.z() + keyframe_h;
        traj_edge_marker.points.push_back(point1);
        traj_edge_marker.points.push_back(point2);

        double p1 = static_cast<double>(v1->id()) / local_graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / local_graph->vertices().size();

        std_msgs::ColorRGBA color1, color2;
        color1.r = 1.0 - p1;
        color1.g = p1;
        color1.a = 1.0;

        color2.r = 1.0 - p2;
        color2.g = p2;
        color2.a = 1.0;
        traj_edge_marker.colors.push_back(color1);
        traj_edge_marker.colors.push_back(color2);

        // if(std::abs(v1->id() - v2->id()) > 2) {
        //   traj_edge_marker.points[i * 2].z += 0.5 + keyframe_h;
        //   traj_edge_marker.points[i * 2 + 1].z += 0.5 + keyframe_h;
        // }
      }
    }
    markers.markers.push_back(traj_edge_marker);

    // keyframe plane edge markers
    visualization_msgs::Marker traj_plane_edge_marker;
    traj_plane_edge_marker.header.frame_id = map_frame_id;
    traj_plane_edge_marker.header.stamp = stamp;
    traj_plane_edge_marker.ns = "keyframe_plane_edges";
    traj_plane_edge_marker.id = markers.markers.size();
    traj_plane_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    traj_plane_edge_marker.pose.orientation.w = 1.0;
    traj_plane_edge_marker.scale.x = 0.01;

    auto traj_plane_edge_itr = local_graph->edges().begin();
    for(int i = 0; traj_plane_edge_itr != local_graph->edges().end(); traj_plane_edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *traj_plane_edge_itr;
      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);

      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_plane->vertices()[1]);

        if(!v1 || !v2) continue;

        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2;

        float r = 0, g = 0, b = 0.0;
        pcl::CentroidPoint<PointNormal> centroid;
        if(fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(1)) && fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(2))) {
          for(auto x_plane : x_plane_snapshot) {
            if(x_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < x_plane.cloud_seg_map->points.size(); ++p) {
                x += x_plane.cloud_seg_map->points[p].x;
                y += x_plane.cloud_seg_map->points[p].y;
              }
              x = x / x_plane.cloud_seg_map->points.size();
              y = y / x_plane.cloud_seg_map->points.size();
              pt2 = Eigen::Vector3d(x, y, 0.0);
            }
          }
          r = 0.0;
        } else if(fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(0)) && fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(2))) {
          for(auto y_plane : y_plane_snapshot) {
            if(y_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < y_plane.cloud_seg_map->points.size(); ++p) {
                x += y_plane.cloud_seg_map->points[p].x;
                y += y_plane.cloud_seg_map->points[p].y;
              }
              x = x / y_plane.cloud_seg_map->points.size();
              y = y / y_plane.cloud_seg_map->points.size();
              pt2 = Eigen::Vector3d(x, y, 0.0);
            }
          }
          b = 0.0;
        } else if(fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(0)) && fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(1))) {
          for(auto h_plane : hort_plane_snapshot) {
            if(h_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < h_plane.cloud_seg_map->points.size(); ++p) {
                x += h_plane.cloud_seg_map->points[p].x;
                y += h_plane.cloud_seg_map->points[p].y;
              }
              x = x / h_plane.cloud_seg_map->points.size();
              y = y / h_plane.cloud_seg_map->points.size();
              pt2 = Eigen::Vector3d(x, y, 0.0);
            }
          }
          r = 0;
          g = 0.0;
        } else
          continue;

        geometry_msgs::Point point1, point2;
        point1.x = pt1.x();
        point1.y = pt1.y();
        point1.z = pt1.z() + keyframe_h;

        point2.x = pt2.x();
        point2.y = pt2.y();
        point2.z = pt2.z() + plane_h;
        traj_plane_edge_marker.points.push_back(point1);
        traj_plane_edge_marker.points.push_back(point2);

        std_msgs::ColorRGBA color1, color2;
        color1.r = 0;
        color1.g = 0;
        color1.b = 0;
        color1.a = 1.0;

        color2.r = 0;
        color2.g = 0;
        color2.b = 0;
        color2.a = 1.0;
        traj_plane_edge_marker.colors.push_back(color1);
        traj_plane_edge_marker.colors.push_back(color2);
      }
    }
    markers.markers.push_back(traj_plane_edge_marker);

    // edge markers
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = map_frame_id;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = markers.markers.size();
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(local_graph->edges().size() * 8);
    edge_marker.colors.resize(local_graph->edges().size() * 8);

    auto edge_itr = local_graph->edges().begin();
    for(int i = 0; edge_itr != local_graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / local_graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / local_graph->vertices().size();
        edge_marker.colors[i * 2].r = 1.0 - p1;
        edge_marker.colors[i * 2].g = p1;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
        edge_marker.colors[i * 2 + 1].g = p2;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i * 2].z += 0.5;
          edge_marker.points[i * 2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_plane->vertices()[1]);

        if(!v1 || !v2) continue;

        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2, pt3;

        float r = 0, g = 0, b = 0.0;
        pcl::CentroidPoint<PointNormal> centroid;
        if(fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(1)) && fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(2))) {
          for(auto x_plane : x_plane_snapshot) {
            if(x_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < x_plane.cloud_seg_map->points.size(); ++p) {
                x += x_plane.cloud_seg_map->points[p].x;
                y += x_plane.cloud_seg_map->points[p].y;
              }
              x = x / x_plane.cloud_seg_map->points.size();
              y = y / x_plane.cloud_seg_map->points.size();
              pt3 = Eigen::Vector3d(x, y, 5.0);
            }
          }
          pt2 = Eigen::Vector3d(pt1.x(), pt1.y(), 3.0);
          r = 0.0;
        } else if(fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(0)) && fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(2))) {
          for(auto y_plane : y_plane_snapshot) {
            if(y_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < y_plane.cloud_seg_map->points.size(); ++p) {
                x += y_plane.cloud_seg_map->points[p].x;
                y += y_plane.cloud_seg_map->points[p].y;
              }
              x = x / y_plane.cloud_seg_map->points.size();
              y = y / y_plane.cloud_seg_map->points.size();
              pt3 = Eigen::Vector3d(x, y, 5.0);
            }
          }
          pt2 = Eigen::Vector3d(pt1.x(), pt1.y(), 3.0);
          b = 0.0;
        } else if(fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(0)) && fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(1))) {
          for(auto h_plane : hort_plane_snapshot) {
            if(h_plane.id == v2->id()) {
              double x = 0, y = 0;
              for(int p = 0; p < h_plane.cloud_seg_map->points.size(); ++p) {
                x += h_plane.cloud_seg_map->points[p].x;
                y += h_plane.cloud_seg_map->points[p].y;
              }
              x = x / h_plane.cloud_seg_map->points.size();
              y = y / h_plane.cloud_seg_map->points.size();
              pt3 = Eigen::Vector3d(x, y, 5.0);
            }
          }
          pt2 = Eigen::Vector3d(pt1.x(), pt1.y(), 3.0);
          r = 0;
          g = 0.0;
        } else
          continue;

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 0;
        edge_marker.colors[i * 2].g = 0;
        edge_marker.colors[i * 2].b = 0.0;
        edge_marker.colors[i * 2].a = 0.4;
        edge_marker.colors[i * 2 + 1].r = 0;
        edge_marker.colors[i * 2 + 1].g = 0;
        edge_marker.colors[i * 2 + 1].b = 0;
        edge_marker.colors[i * 2 + 1].a = 0.4;

        edge_marker.points[(local_graph->edges().size() * 2) + i * 2].x = pt2.x();
        edge_marker.points[(local_graph->edges().size() * 2) + i * 2].y = pt2.y();
        edge_marker.points[(local_graph->edges().size() * 2) + i * 2].z = pt2.z();
        edge_marker.points[(local_graph->edges().size() * 2) + (i * 2 + 1)].x = pt3.x();
        edge_marker.points[(local_graph->edges().size() * 2) + (i * 2 + 1)].y = pt3.y();
        edge_marker.points[(local_graph->edges().size() * 2) + (i * 2 + 1)].z = pt3.z();

        edge_marker.colors[(local_graph->edges().size() * 2) + i * 2].r = r;
        edge_marker.colors[(local_graph->edges().size() * 2) + i * 2].g = g;
        edge_marker.colors[(local_graph->edges().size() * 2) + i * 2].b = b;
        edge_marker.colors[(local_graph->edges().size() * 2) + i * 2].a = 0.4;
        edge_marker.colors[(local_graph->edges().size() * 2) + (i * 2 + 1)].r = r;
        edge_marker.colors[(local_graph->edges().size() * 2) + (i * 2 + 1)].g = g;
        edge_marker.colors[(local_graph->edges().size() * 2) + (i * 2 + 1)].b = b;
        edge_marker.colors[(local_graph->edges().size() * 2) + (i * 2 + 1)].a = 0.4;

        continue;
      }

      g2o::EdgeSE3PointToPlane* edge_point_to_plane = dynamic_cast<g2o::EdgeSE3PointToPlane*>(edge);
      if(edge_point_to_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_point_to_plane->vertices()[0]);
        g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_point_to_plane->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2;
        float r = 0, g = 0, b = 0.0;
        double x = 0, y = 0;
        if(fabs(v2->estimate().normal()(0)) > 0.95) {
          for(auto x_plane : x_plane_snapshot) {
            if(x_plane.id == v2->id()) {
              x = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size() / 2)].x;
              y = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size() / 2)].y;
            }
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          r = 1.0;
        } else if(fabs(v2->estimate().normal()(1)) > 0.95) {
          for(auto y_plane : y_plane_snapshot) {
            if(y_plane.id == v2->id()) {
              x = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size() / 2)].x;
              y = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size() / 2)].y;
            }
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          b = 1.0;
        } else if(fabs(v2->estimate().normal()(2)) > 0.95) {
          for(auto h_plane : hort_plane_snapshot) {
            if(h_plane.id == v2->id()) {
              x = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size() / 2)].x;
              y = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size() / 2)].y;
            }
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          r = 1;
          g = 0.65;
        }

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = r;
        edge_marker.colors[i * 2].g = g;
        edge_marker.colors[i * 2].b = b;
        edge_marker.colors[i * 2].a = 0.4;
        edge_marker.colors[i * 2 + 1].r = r;
        edge_marker.colors[i * 2 + 1].g = g;
        edge_marker.colors[i * 2 + 1].b = b;
        edge_marker.colors[i * 2 + 1].a = 0.4;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }
    markers.markers.push_back(edge_marker);

    // sphere
    visualization_msgs::Marker sphere_marker;
    sphere_marker.header.frame_id = map_frame_id;
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = markers.markers.size();
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    if(!keyframes.empty()) {
      Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z() + keyframe_h;
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;
    markers.markers.push_back(sphere_marker);

    // x vertical plane markers
    visualization_msgs::Marker x_vert_plane_marker;
    x_vert_plane_marker.pose.orientation.w = 1.0;
    x_vert_plane_marker.scale.x = 0.05;
    x_vert_plane_marker.scale.y = 0.05;
    x_vert_plane_marker.scale.z = 0.05;
    // plane_marker.points.resize(vert_planes.size());
    x_vert_plane_marker.header.frame_id = map_frame_id;
    x_vert_plane_marker.header.stamp = stamp;
    x_vert_plane_marker.ns = "x_vert_planes";
    x_vert_plane_marker.id = markers.markers.size();
    x_vert_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;

    for(int i = 0; i < x_plane_snapshot.size(); ++i) {
      double p = static_cast<double>(i) / x_plane_snapshot.size();
      std_msgs::ColorRGBA color;
      color.r = x_plane_snapshot[i].color[0] / 255;
      color.g = x_plane_snapshot[i].color[1] / 255;
      color.b = x_plane_snapshot[i].color[2] / 255;
      color.a = 0.5;
      for(size_t j = 0; j < x_plane_snapshot[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = x_plane_snapshot[i].cloud_seg_map->points[j].x;
        point.y = x_plane_snapshot[i].cloud_seg_map->points[j].y;
        point.z = x_plane_snapshot[i].cloud_seg_map->points[j].z + plane_h;
        x_vert_plane_marker.points.push_back(point);
        x_vert_plane_marker.colors.push_back(color);
      }
    }
    markers.markers.push_back(x_vert_plane_marker);

    // y vertical plane markers
    visualization_msgs::Marker y_vert_plane_marker;
    y_vert_plane_marker.pose.orientation.w = 1.0;
    y_vert_plane_marker.scale.x = 0.05;
    y_vert_plane_marker.scale.y = 0.05;
    y_vert_plane_marker.scale.z = 0.05;
    // plane_marker.points.resize(vert_planes.size());
    y_vert_plane_marker.header.frame_id = map_frame_id;
    y_vert_plane_marker.header.stamp = stamp;
    y_vert_plane_marker.ns = "y_vert_planes";
    y_vert_plane_marker.id = markers.markers.size();
    y_vert_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;

    for(int i = 0; i < y_plane_snapshot.size(); ++i) {
      double p = static_cast<double>(i) / y_plane_snapshot.size();
      std_msgs::ColorRGBA color;
      color.r = y_plane_snapshot[i].color[0] / 255;
      color.g = y_plane_snapshot[i].color[1] / 255;
      color.b = y_plane_snapshot[i].color[2] / 255;
      color.a = 0.5;
      for(size_t j = 0; j < y_plane_snapshot[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = y_plane_snapshot[i].cloud_seg_map->points[j].x;
        point.y = y_plane_snapshot[i].cloud_seg_map->points[j].y;
        point.z = y_plane_snapshot[i].cloud_seg_map->points[j].z + plane_h;
        y_vert_plane_marker.points.push_back(point);
        y_vert_plane_marker.colors.push_back(color);
      }
    }
    markers.markers.push_back(y_vert_plane_marker);

    // horizontal plane markers
    visualization_msgs::Marker hort_plane_marker;
    hort_plane_marker.pose.orientation.w = 1.0;
    hort_plane_marker.scale.x = 0.05;
    hort_plane_marker.scale.y = 0.05;
    hort_plane_marker.scale.z = 0.05;
    // plane_marker.points.resize(vert_planes.size());
    hort_plane_marker.header.frame_id = map_frame_id;
    hort_plane_marker.header.stamp = stamp;
    hort_plane_marker.ns = "hort_planes";
    hort_plane_marker.id = markers.markers.size();
    hort_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;

    for(int i = 0; i < hort_plane_snapshot.size(); ++i) {
      for(size_t j = 0; j < hort_plane_snapshot[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = hort_plane_snapshot[i].cloud_seg_map->points[j].x;
        point.y = hort_plane_snapshot[i].cloud_seg_map->points[j].y;
        point.z = hort_plane_snapshot[i].cloud_seg_map->points[j].z + plane_h;
        hort_plane_marker.points.push_back(point);
      }
      hort_plane_marker.color.r = 1;
      hort_plane_marker.color.g = 0.65;
      hort_plane_marker.color.a = 0.5;
    }
    markers.markers.push_back(hort_plane_marker);

    float corridor_node_h = 15;
    float corridor_text_h = 12;
    float corridor_edge_h = 14.5;
    float corridor_point_h = 10.0;

    for(int i = 0; i < x_corridor_snapshot.size(); ++i) {
      bool overlapped_corridor = false;
      float dist_room_x_corr = 100;
      for(const auto& room : room_snapshot) {
        if((room.plane_x1_id == x_corridor_snapshot[i].plane1_id || room.plane_x1_id == x_corridor_snapshot[i].plane2_id) && (room.plane_x2_id == x_corridor_snapshot[i].plane1_id || room.plane_x2_id == x_corridor_snapshot[i].plane2_id)) {
          overlapped_corridor = true;
          break;
        }
        dist_room_x_corr = sqrt(pow(room.node->estimate()(0) - x_corridor_snapshot[i].node->estimate(), 2) + pow(room.node->estimate()(1) - x_corridor_snapshot[i].keyframe_trans(1), 2));
        if(dist_room_x_corr < 1.0) {
          overlapped_corridor = true;
          break;
        }
      }

      auto found_plane1 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridor_snapshot[i].plane1_id);
      auto found_plane2 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridor_snapshot[i].plane2_id);

      // fill in the line marker
      visualization_msgs::Marker corr_x_line_marker;
      corr_x_line_marker.scale.x = 0.04;
      corr_x_line_marker.pose.orientation.w = 1.0;
      if(!overlapped_corridor)
        corr_x_line_marker.ns = "corridor_x_lines";
      else {
        x_corridor_snapshot[i].id = -1;
        corr_x_line_marker.ns = "overlapped_corridor_x_lines";
      }
      corr_x_line_marker.header.frame_id = map_frame_id;
      corr_x_line_marker.header.stamp = stamp;
      corr_x_line_marker.id = markers.markers.size() + 1;
      corr_x_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      corr_x_line_marker.color.r = color_r;
      corr_x_line_marker.color.g = color_g;
      corr_x_line_marker.color.b = color_b;
      corr_x_line_marker.color.a = 1.0;
      corr_x_line_marker.lifetime = ros::Duration(15.0);

      geometry_msgs::Point p1, p2, p3;
      p1.x = x_corridor_snapshot[i].node->estimate();
      p1.y = x_corridor_snapshot[i].keyframe_trans(1);
      p1.z = corridor_edge_h;

      float min_dist_plane1 = 100;
      for(int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_plane1).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_plane1).cloud_seg_map->points[p].y;
        p_tmp.z = corridor_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_plane1) {
          min_dist_plane1 = norm;
          p2 = p_tmp;
        }
      }
      corr_x_line_marker.points.push_back(p1);
      corr_x_line_marker.points.push_back(p2);

      float min_dist_plane2 = 100;
      for(int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_plane2).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_plane2).cloud_seg_map->points[p].y;
        p_tmp.z = corridor_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_plane2) {
          min_dist_plane2 = norm;
          p3 = p_tmp;
        }
      }
      corr_x_line_marker.points.push_back(p1);
      corr_x_line_marker.points.push_back(p3);
      markers.markers.push_back(corr_x_line_marker);

      // x corridor cube
      visualization_msgs::Marker corridor_pose_marker;
      corridor_pose_marker.pose.orientation.w = 1.0;
      corridor_pose_marker.scale.x = 0.5;
      corridor_pose_marker.scale.y = 0.5;
      corridor_pose_marker.scale.z = 0.5;
      // plane_marker.points.resize(vert_planes.size());
      corridor_pose_marker.header.frame_id = map_frame_id;
      corridor_pose_marker.header.stamp = stamp;
      if(!overlapped_corridor)
        corridor_pose_marker.ns = "x_corridor";
      else
        corridor_pose_marker.ns = "overlapped_x_corridor";
      corridor_pose_marker.id = markers.markers.size();
      corridor_pose_marker.type = visualization_msgs::Marker::CUBE;
      corridor_pose_marker.color.r = 1;
      corridor_pose_marker.color.g = 0.64;
      corridor_pose_marker.color.a = 1;
      corridor_pose_marker.pose.position.x = x_corridor_snapshot[i].node->estimate();
      corridor_pose_marker.pose.position.y = x_corridor_snapshot[i].keyframe_trans(1);
      corridor_pose_marker.pose.position.z = corridor_node_h;
      corridor_pose_marker.lifetime = ros::Duration(15.0);
      markers.markers.push_back(corridor_pose_marker);
    }

    for(int i = 0; i < y_corridor_snapshot.size(); ++i) {
      bool overlapped_corridor = false;
      float dist_room_y_corr = 100;
      for(const auto& room : room_snapshot) {
        if((room.plane_y1_id == y_corridor_snapshot[i].plane1_id || room.plane_y1_id == y_corridor_snapshot[i].plane2_id) || (room.plane_y2_id == y_corridor_snapshot[i].plane1_id || room.plane_y2_id == y_corridor_snapshot[i].plane2_id)) {
          overlapped_corridor = true;
          break;
        }
        dist_room_y_corr = sqrt(pow(room.node->estimate()(0) - y_corridor_snapshot[i].keyframe_trans(0), 2) + pow(room.node->estimate()(1) - y_corridor_snapshot[i].node->estimate(), 2));
        if(dist_room_y_corr < 1.0) {
          overlapped_corridor = true;
          break;
        }
      }

      auto found_plane1 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridor_snapshot[i].plane1_id);
      auto found_plane2 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridor_snapshot[i].plane2_id);

      // fill in the line marker
      visualization_msgs::Marker corr_y_line_marker;
      corr_y_line_marker.scale.x = 0.04;
      corr_y_line_marker.pose.orientation.w = 1.0;
      if(!overlapped_corridor)
        corr_y_line_marker.ns = "corridor_y_lines";
      else {
        y_corridor_snapshot[i].id = -1;
        corr_y_line_marker.ns = "overlapped_corridor_y_lines";
      }
      corr_y_line_marker.header.frame_id = map_frame_id;
      corr_y_line_marker.header.stamp = stamp;
      corr_y_line_marker.id = markers.markers.size() + 1;
      corr_y_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      corr_y_line_marker.color.r = color_r;
      corr_y_line_marker.color.g = color_g;
      corr_y_line_marker.color.b = color_b;
      corr_y_line_marker.color.a = 1.0;
      corr_y_line_marker.lifetime = ros::Duration(15.0);

      geometry_msgs::Point p1, p2, p3;
      p1.x = y_corridor_snapshot[i].keyframe_trans(0);
      p1.y = y_corridor_snapshot[i].node->estimate();
      p1.z = corridor_edge_h;

      float min_dist_plane1 = 100;
      for(int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_plane1).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_plane1).cloud_seg_map->points[p].y;
        p_tmp.z = corridor_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_plane1) {
          min_dist_plane1 = norm;
          p2 = p_tmp;
        }
      }
      corr_y_line_marker.points.push_back(p1);
      corr_y_line_marker.points.push_back(p2);

      float min_dist_plane2 = 100;
      for(int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_plane2).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_plane2).cloud_seg_map->points[p].y;
        p_tmp.z = corridor_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_plane2) {
          min_dist_plane2 = norm;
          p3 = p_tmp;
        }
      }
      corr_y_line_marker.points.push_back(p1);
      corr_y_line_marker.points.push_back(p3);
      markers.markers.push_back(corr_y_line_marker);

      // y corridor cube
      visualization_msgs::Marker corridor_pose_marker;
      corridor_pose_marker.pose.orientation.w = 1.0;
      corridor_pose_marker.scale.x = 0.5;
      corridor_pose_marker.scale.y = 0.5;
      corridor_pose_marker.scale.z = 0.5;
      // plane_marker.points.resize(vert_planes.size());
      corridor_pose_marker.header.frame_id = map_frame_id;
      corridor_pose_marker.header.stamp = stamp;
      if(!overlapped_corridor)
        corridor_pose_marker.ns = "y_corridor";
      else
        corridor_pose_marker.ns = "overlapped_y_corridor";
      corridor_pose_marker.id = markers.markers.size();
      corridor_pose_marker.type = visualization_msgs::Marker::CUBE;
      corridor_pose_marker.color.r = 0.13;
      corridor_pose_marker.color.g = 0.54;
      corridor_pose_marker.color.b = 0.13;
      corridor_pose_marker.color.a = 1;
      corridor_pose_marker.pose.position.x = y_corridor_snapshot[i].keyframe_trans(0);
      corridor_pose_marker.pose.position.y = y_corridor_snapshot[i].node->estimate();
      corridor_pose_marker.pose.position.z = corridor_node_h;
      corridor_pose_marker.lifetime = ros::Duration(15.0);
      markers.markers.push_back(corridor_pose_marker);
    }

    // room markers
    float room_node_h = 15;
    float room_text_h = 12;
    float room_edge_h = 14.5;
    float room_point_h = 10.0;
    visualization_msgs::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = map_frame_id;
    room_marker.header.stamp = stamp;
    room_marker.ns = "rooms";
    room_marker.id = markers.markers.size();
    room_marker.type = visualization_msgs::Marker::CUBE_LIST;
    room_marker.color.r = 1;
    room_marker.color.g = 0.07;
    room_marker.color.b = 0.57;
    room_marker.color.a = 1;
    room_marker.lifetime = ros::Duration(15.0);

    for(int i = 0; i < room_snapshot.size(); ++i) {
      geometry_msgs::Point point;
      point.x = room_snapshot[i].node->estimate()(0);
      point.y = room_snapshot[i].node->estimate()(1);
      point.z = room_node_h;
      room_marker.points.push_back(point);

      // fill in the line marker
      visualization_msgs::Marker room_line_marker;
      room_line_marker.scale.x = 0.04;
      room_line_marker.pose.orientation.w = 1.0;
      room_line_marker.ns = "rooms_lines";
      room_line_marker.header.frame_id = map_frame_id;
      room_line_marker.header.stamp = stamp;
      room_line_marker.id = markers.markers.size() + 1;
      room_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      room_line_marker.color.r = color_r;
      room_line_marker.color.g = color_g;
      room_line_marker.color.b = color_b;
      room_line_marker.color.a = 1.0;
      room_line_marker.lifetime = ros::Duration(15.0);
      geometry_msgs::Point p1, p2, p3, p4, p5;
      p1.x = room_snapshot[i].node->estimate()(0);
      p1.y = room_snapshot[i].node->estimate()(1);
      p1.z = room_edge_h;

      auto found_planex1 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_x1_id);
      auto found_planex2 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_x2_id);
      auto found_planey1 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_y1_id);
      auto found_planey2 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_y2_id);

      float min_dist_x1 = 100;
      for(int p = 0; p < (*found_planex1).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_planex1).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_planex1).cloud_seg_map->points[p].y;
        p_tmp.z = room_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_x1) {
          min_dist_x1 = norm;
          p2 = p_tmp;
        }
      }
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p2);

      float min_dist_x2 = 100;
      for(int p = 0; p < (*found_planex2).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_planex2).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_planex2).cloud_seg_map->points[p].y;
        p_tmp.z = room_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_x2) {
          min_dist_x2 = norm;
          p3 = p_tmp;
        }
      }
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p3);

      float min_dist_y1 = 100;
      for(int p = 0; p < (*found_planey1).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_planey1).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_planey1).cloud_seg_map->points[p].y;
        p_tmp.z = room_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_y1) {
          min_dist_y1 = norm;
          p4 = p_tmp;
        }
      }
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p4);

      float min_dist_y2 = 100;
      for(int p = 0; p < (*found_planey2).cloud_seg_map->points.size(); ++p) {
        geometry_msgs::Point p_tmp;
        p_tmp.x = (*found_planey2).cloud_seg_map->points[p].x;
        p_tmp.y = (*found_planey2).cloud_seg_map->points[p].y;
        p_tmp.z = room_point_h;

        float norm = std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) + std::pow((p1.z - p_tmp.z), 2));

        if(norm < min_dist_y2) {
          min_dist_y2 = norm;
          p5 = p_tmp;
        }
      }
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p5);

      markers.markers.push_back(room_line_marker);
    }
    markers.markers.push_back(room_marker);

    // check for xcorridor neighbours and draw lines between them
    visualization_msgs::Marker x_corr_neighbour_line_marker;
    x_corr_neighbour_line_marker.scale.x = 0.02;
    x_corr_neighbour_line_marker.pose.orientation.w = 1.0;
    x_corr_neighbour_line_marker.ns = "x_corr_neighbour_lines";
    x_corr_neighbour_line_marker.header.frame_id = map_frame_id;
    x_corr_neighbour_line_marker.header.stamp = stamp;
    x_corr_neighbour_line_marker.id = markers.markers.size() + 1;
    x_corr_neighbour_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    x_corr_neighbour_line_marker.color.r = 1;
    x_corr_neighbour_line_marker.color.g = 0;
    x_corr_neighbour_line_marker.color.b = 0;
    x_corr_neighbour_line_marker.color.a = 1.0;
    x_corr_neighbour_line_marker.lifetime = ros::Duration(15.0);

    for(const auto& x_corridor : x_corridor_snapshot) {
      for(const auto& x_corridor_neighbour_id : x_corridor.neighbour_ids) {
        geometry_msgs::Point p1, p2;
        p1.x = x_corridor.node->estimate();
        p1.y = x_corridor.keyframe_trans(1);
        p1.z = corridor_node_h;

        auto found_neighbour_room = std::find_if(room_snapshot.begin(), room_snapshot.end(), boost::bind(&Rooms::id, _1) == x_corridor_neighbour_id);
        if(found_neighbour_room != room_snapshot.end()) {
          p2.x = (*found_neighbour_room).node->estimate()(0);
          p2.y = (*found_neighbour_room).node->estimate()(1);
          p2.z = room_node_h;

          x_corr_neighbour_line_marker.points.push_back(p1);
          x_corr_neighbour_line_marker.points.push_back(p2);
        } else {
          auto found_neighbour_x_corr = std::find_if(x_corridor_snapshot.begin(), x_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == x_corridor_neighbour_id);
          if(found_neighbour_x_corr != x_corridor_snapshot.end()) {
            p2.x = (*found_neighbour_x_corr).node->estimate();
            p2.y = (*found_neighbour_x_corr).keyframe_trans(1);
            p2.z = corridor_node_h;

            x_corr_neighbour_line_marker.points.push_back(p1);
            x_corr_neighbour_line_marker.points.push_back(p2);
          } else {
            auto found_neighbour_y_corr = std::find_if(y_corridor_snapshot.begin(), y_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == x_corridor_neighbour_id);
            if(found_neighbour_y_corr != y_corridor_snapshot.end()) {
              p2.x = (*found_neighbour_y_corr).keyframe_trans(0);
              p2.y = (*found_neighbour_y_corr).node->estimate();
              p2.z = corridor_node_h;

              x_corr_neighbour_line_marker.points.push_back(p1);
              x_corr_neighbour_line_marker.points.push_back(p2);
            } else
              continue;
          }
        }
      }
    }
    markers.markers.push_back(x_corr_neighbour_line_marker);

    // check for ycorridor neighbours and draw lines between them
    visualization_msgs::Marker y_corr_neighbour_line_marker;
    y_corr_neighbour_line_marker.scale.x = 0.02;
    y_corr_neighbour_line_marker.pose.orientation.w = 1.0;
    y_corr_neighbour_line_marker.ns = "y_corr_neighbour_lines";
    y_corr_neighbour_line_marker.header.frame_id = map_frame_id;
    y_corr_neighbour_line_marker.header.stamp = stamp;
    y_corr_neighbour_line_marker.id = markers.markers.size() + 1;
    y_corr_neighbour_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    y_corr_neighbour_line_marker.color.r = 1;
    y_corr_neighbour_line_marker.color.g = 0;
    y_corr_neighbour_line_marker.color.b = 0;
    y_corr_neighbour_line_marker.color.a = 1.0;
    y_corr_neighbour_line_marker.lifetime = ros::Duration(15.0);

    for(const auto& y_corridor : y_corridor_snapshot) {
      for(const auto& y_corridor_neighbour_id : y_corridor.neighbour_ids) {
        geometry_msgs::Point p1, p2;
        p1.x = y_corridor.keyframe_trans(0);
        p1.y = y_corridor.node->estimate();
        p1.z = corridor_node_h;

        auto found_neighbour_room = std::find_if(room_snapshot.begin(), room_snapshot.end(), boost::bind(&Rooms::id, _1) == y_corridor_neighbour_id);
        if(found_neighbour_room != room_snapshot.end()) {
          p2.x = (*found_neighbour_room).node->estimate()(0);
          p2.y = (*found_neighbour_room).node->estimate()(1);
          p2.z = room_node_h;

          y_corr_neighbour_line_marker.points.push_back(p1);
          y_corr_neighbour_line_marker.points.push_back(p2);
        } else {
          auto found_neighbour_x_corr = std::find_if(x_corridor_snapshot.begin(), x_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == y_corridor_neighbour_id);
          if(found_neighbour_x_corr != x_corridor_snapshot.end()) {
            p2.x = (*found_neighbour_x_corr).node->estimate();
            p2.y = (*found_neighbour_x_corr).keyframe_trans(1);
            p2.z = corridor_node_h;

            y_corr_neighbour_line_marker.points.push_back(p1);
            y_corr_neighbour_line_marker.points.push_back(p2);
          } else {
            auto found_neighbour_y_corr = std::find_if(y_corridor_snapshot.begin(), y_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == y_corridor_neighbour_id);
            if(found_neighbour_y_corr != y_corridor_snapshot.end()) {
              p2.x = (*found_neighbour_y_corr).keyframe_trans(0);
              p2.y = (*found_neighbour_y_corr).node->estimate();
              p2.z = corridor_node_h;

              y_corr_neighbour_line_marker.points.push_back(p1);
              y_corr_neighbour_line_marker.points.push_back(p2);
            } else
              continue;
          }
        }
      }
    }
    markers.markers.push_back(y_corr_neighbour_line_marker);

    // check the neighbours for the rooms and draw lines between them
    visualization_msgs::Marker room_neighbour_line_marker;
    room_neighbour_line_marker.scale.x = 0.02;
    room_neighbour_line_marker.pose.orientation.w = 1.0;
    room_neighbour_line_marker.ns = "room_neighbour_lines";
    room_neighbour_line_marker.header.frame_id = map_frame_id;
    room_neighbour_line_marker.header.stamp = stamp;
    room_neighbour_line_marker.id = markers.markers.size() + 1;
    room_neighbour_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    room_neighbour_line_marker.color.r = 1;
    room_neighbour_line_marker.color.g = 0;
    room_neighbour_line_marker.color.b = 0;
    room_neighbour_line_marker.color.a = 1.0;
    room_neighbour_line_marker.lifetime = ros::Duration(15.0);

    for(const auto& room : room_snapshot) {
      for(const auto& room_neighbour_id : room.neighbour_ids) {
        geometry_msgs::Point p1, p2;
        p1.x = room.node->estimate()(0);
        p1.y = room.node->estimate()(1);
        p1.z = room_node_h;

        auto found_neighbour_room = std::find_if(room_snapshot.begin(), room_snapshot.end(), boost::bind(&Rooms::id, _1) == room_neighbour_id);
        if(found_neighbour_room != room_snapshot.end()) {
          p2.x = (*found_neighbour_room).node->estimate()(0);
          p2.y = (*found_neighbour_room).node->estimate()(1);
          p2.z = room_node_h;

          room_neighbour_line_marker.points.push_back(p1);
          room_neighbour_line_marker.points.push_back(p2);
        } else {
          auto found_neighbour_x_corr = std::find_if(x_corridor_snapshot.begin(), x_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == room_neighbour_id);
          if(found_neighbour_x_corr != x_corridor_snapshot.end()) {
            p2.x = (*found_neighbour_x_corr).node->estimate();
            p2.y = (*found_neighbour_x_corr).keyframe_trans(1);
            p2.z = corridor_node_h;
            room_neighbour_line_marker.points.push_back(p1);
            room_neighbour_line_marker.points.push_back(p2);
          } else {
            auto found_neighbour_y_corr = std::find_if(y_corridor_snapshot.begin(), y_corridor_snapshot.end(), boost::bind(&Corridors::id, _1) == room_neighbour_id);
            if(found_neighbour_y_corr != y_corridor_snapshot.end()) {
              p2.x = (*found_neighbour_y_corr).keyframe_trans(0);
              p2.y = (*found_neighbour_y_corr).node->estimate();
              p2.z = corridor_node_h;
              room_neighbour_line_marker.points.push_back(p1);
              room_neighbour_line_marker.points.push_back(p2);
            } else
              continue;
          }
        }
      }
    }
    markers.markers.push_back(room_neighbour_line_marker);

    if(floor_center_data.id != -1) {
      float floor_node_h = 20;
      float floor_edge_h = 19.5;
      visualization_msgs::Marker floor_marker;
      floor_marker.pose.orientation.w = 1.0;
      floor_marker.scale.x = 0.5;
      floor_marker.scale.y = 0.5;
      floor_marker.scale.z = 0.5;
      // plane_marker.points.resize(vert_planes.size());
      floor_marker.header.frame_id = map_frame_id;
      floor_marker.header.stamp = stamp;
      floor_marker.ns = "floors";
      floor_marker.id = markers.markers.size();
      floor_marker.type = visualization_msgs::Marker::CUBE;
      floor_marker.color.r = 0.49;
      floor_marker.color.g = 0;
      floor_marker.color.b = 1;
      floor_marker.color.a = 1;
      floor_marker.lifetime = ros::Duration(10.0);

      floor_marker.pose.position.x = floor_center_data.room_center.x;
      floor_marker.pose.position.y = floor_center_data.room_center.y;
      floor_marker.pose.position.z = floor_node_h;

      // create line markers between floor and rooms/corridors
      visualization_msgs::Marker floor_line_marker;
      floor_line_marker.scale.x = 0.04;
      floor_line_marker.pose.orientation.w = 1.0;
      floor_line_marker.ns = "rooms_lines";
      floor_line_marker.header.frame_id = map_frame_id;
      floor_line_marker.header.stamp = stamp;
      floor_line_marker.id = markers.markers.size() + 1;
      floor_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      floor_line_marker.color.r = color_r;
      floor_line_marker.color.g = color_g;
      floor_line_marker.color.b = color_b;
      floor_line_marker.color.a = 1.0;
      floor_line_marker.lifetime = ros::Duration(10.0);

      for(const auto& room : room_snapshot) {
        geometry_msgs::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_node_h;
        p2.x = room.node->estimate()(0);
        p2.y = room.node->estimate()(1);
        p2.z = room_node_h;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for(const auto& x_corridor : x_corridor_snapshot) {
        if(x_corridor.id == -1) continue;
        geometry_msgs::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_node_h;
        p2.x = x_corridor.node->estimate();
        p2.y = x_corridor.keyframe_trans(1);
        p2.z = corridor_node_h;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for(const auto& y_corridor : y_corridor_snapshot) {
        if(y_corridor.id == -1) continue;
        geometry_msgs::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_node_h;
        p2.x = y_corridor.keyframe_trans(0);
        p2.y = y_corridor.node->estimate();
        p2.z = corridor_node_h;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      markers.markers.push_back(floor_marker);
      markers.markers.push_back(floor_line_marker);
    }

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(s_graphs::DumpGraphRequest& req, s_graphs::DumpGraphResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req.destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(int i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
    ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

    res.success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(s_graphs::SaveMapRequest& req, s_graphs::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    if(zero_utm && req.utm) {
      for(auto& pt : cloud->points) {
        pt.getVector3fMap() += (*zero_utm).cast<float>();
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req.destination + ".utm");
      ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::Timer optimization_timer;
  ros::Timer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub;

  ros::Subscriber raw_odom_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;
  ros::Subscriber room_data_sub;
  ros::Subscriber all_room_data_sub;
  ros::Subscriber floor_data_sub;
  ros::Subscriber init_odom2map_sub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  bool wait_trans_odom2map, got_trans_odom2map;
  std::vector<geometry_msgs::PoseStamped> odom_path_vec;
  std::string map_frame_id;
  std::string odom_frame_id;
  std::string points_topic;

  ros::Publisher markers_pub;
  ros::Publisher odom2map_pub;
  ros::Publisher odom_pose_corrected_pub;
  ros::Publisher odom_path_corrected_pub;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;
  ros::Publisher map_planes_pub;
  ros::Publisher all_map_planes_pub;

  tf::TransformListener tf_listener;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

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
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;

  // floor_coeffs queue
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<s_graphs::FloorCoeffsConstPtr> floor_coeffs_queue;

  // vertical and horizontal planes
  int keyframe_window_size;
  bool extract_planar_surfaces;
  double plane_dist_threshold;
  double plane_points_dist;
  bool constant_covariance;
  double min_plane_points;
  bool use_point_to_plane;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  bool use_corridor_constraint, use_room_constraint;
  double corridor_information;
  double corridor_dist_threshold, corridor_min_plane_length, corridor_min_width, corridor_max_width;
  double corridor_plane_length_diff_threshold, corridor_point_diff_threshold;
  double corridor_min_seg_dist;
  double room_information;
  double room_plane_length_diff_threshold, room_point_diff_threshold;
  double room_dist_threshold, room_min_plane_length, room_max_plane_length, room_min_width, room_max_width;
  double room_width_diff_threshold;
  double color_r, color_g, color_b;
  std::vector<VerticalPlanes> x_vert_planes, y_vert_planes;                                      // vertically segmented planes
  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes, dupl_y_vert_planes;  // vertically segmented planes
  std::vector<HorizontalPlanes> hort_planes;                                                     // horizontally segmented planes
  int vertex_count;
  std::vector<Corridors> x_corridors, y_corridors;  // corridors segmented from planes
  std::vector<Rooms> rooms_vec;                     // rooms segmented from planes

  std::mutex vert_plane_snapshot_mutex;
  std::vector<VerticalPlanes> x_vert_planes_snapshot, y_vert_planes_snapshot;  // snapshot of vertically segmented planes

  std::mutex hort_plane_snapshot_mutex;
  std::vector<HorizontalPlanes> hort_planes_snapshot;

  std::mutex corridor_snapshot_mutex;
  std::vector<Corridors> x_corridors_snapshot, y_corridors_snapshot;

  std::mutex room_snapshot_mutex;
  std::vector<Rooms> rooms_vec_snapshot;

  // Seg map queue
  std::mutex cloud_seg_mutex;
  std::deque<s_graphs::PointClouds::Ptr> clouds_seg_queue;

  // room data queue
  std::mutex room_data_queue_mutex;
  std::deque<s_graphs::RoomsData> room_data_queue;

  // all room data queue
  std::mutex all_room_data_queue_mutex;
  std::deque<s_graphs::RoomsData> all_room_data_queue;

  std::mutex floor_data_mutex;
  s_graphs::RoomData floor_center_data;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  std::mutex graph_mutex;
  g2o::SparseOptimizer* graph_snapshot;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<PlaneAnalyzer> plane_analyzer;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<PlaneUtils> plane_utils;
  std::unique_ptr<PlaneMapper> plane_mapper;
  std::unique_ptr<InfiniteRoomMapper> inf_room_mapper;
  std::unique_ptr<FiniteRoomMapper> finite_room_mapper;
  std::unique_ptr<NeighbourMapper> neighbour_mapper;
};

}  // namespace s_graphs

PLUGINLIB_EXPORT_CLASS(s_graphs::SGraphsNodelet, nodelet::Nodelet)