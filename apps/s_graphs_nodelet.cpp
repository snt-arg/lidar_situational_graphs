// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>

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

  struct plane_data_list {
    g2o::Plane3D plane_local;
    // g2o::Plane3D plane;
    g2o::Plane3D plane_unflipped;
    int plane_id;
    pcl::PointXY start_point, end_point;
    float plane_length;
    g2o::VertexSE3* keyframe_node;
    Eigen::Vector3d plane_centroid;
    int connected_id;
  };

  struct structure_data_list {
    plane_data_list plane1;
    plane_data_list plane2;
    float width;
    float length_diff;
    float avg_point_diff;
  };

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

    //
    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    plane_analyzer.reset(new PlaneAnalyzer(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);

    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

    plane_dist_threshold = private_nh.param<double>("plane_dist_threshold", 0.15);
    constant_covariance = private_nh.param<bool>("constant_covariance", true);
    min_plane_points = private_nh.param<double>("min_plane_points", 100);
    use_point_to_plane = private_nh.param<bool>("use_point_to_plane", false);
    use_parallel_plane_constraint = private_nh.param<bool>("use_parallel_plane_constraint", true);
    use_perpendicular_plane_constraint = private_nh.param<bool>("use_perpendicular_plane_constraint", true);

    use_corridor_constraint = private_nh.param<bool>("use_corridor_constraint", false);
    corridor_information = private_nh.param<double>("corridor_information", 0.01);
    corridor_dist_threshold = private_nh.param<double>("corridor_dist_threshold", 1.0);
    corridor_min_plane_length = private_nh.param<double>("corridor_min_plane_length", 10);
    corridor_min_width = private_nh.param<double>("corridor_min_width", 1.5);
    corridor_max_width = private_nh.param<double>("corridor_max_width", 2.5);
    corridor_plane_length_diff_threshold = private_nh.param<double>("corridor_plane_length_diff_threshold", 0.3);
    corridor_point_diff_threshold = private_nh.param<double>("corridor_point_diff_threshold", 3.0);

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
    keyframe_window_size = private_nh.param<int>("keyframe_window_size", 1);

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
    cloud_seg_sub = nh.subscribe("/segmented_clouds", 32, &SGraphsNodelet::cloud_seg_callback, this);
    room_data_sub = nh.subscribe("/room_segmentation/room_data", 1, &SGraphsNodelet::room_data_callback, this);

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &SGraphsNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &SGraphsNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &SGraphsNodelet::navsat_callback, this);
    }

    // publishers
    map_planes_pub = mt_nh.advertise<s_graphs::PlanesData>("/s_graphs/map_planes", 1, false);
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/s_graphs/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/s_graphs/odom2map", 16);
    odom_pose_corrected_pub = mt_nh.advertise<geometry_msgs::PoseStamped>("/s_graphs/odom_pose_corrected", 10);
    odom_path_corrected_pub = mt_nh.advertise<nav_msgs::Path>("/s_graphs/odom_path_corrected", 10);

    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/s_graphs/map_points", 1, true);
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/s_graphs/read_until", 32);

    dump_service_server = mt_nh.advertiseService("/s_graphs/dump", &SGraphsNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/s_graphs/save_map", &SGraphsNodelet::save_map_service, this);

    graph_updated = false;
    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &SGraphsNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &SGraphsNodelet::map_points_publish_timer_callback, this);
  }

private:
  /**
   * @brief receive the raw odom msg to publish the corrected odom after
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
   * @brief received segmented clouds pushed to be pushed #keyframe_queue
   * @param clouds_seg_msg
   */
  void cloud_seg_callback(const s_graphs::PointClouds::Ptr& clouds_seg_msg) {
    std::lock_guard<std::mutex> lock(cloud_seg_mutex);
    clouds_seg_queue.push_back(clouds_seg_msg);
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

  /**
   * @brief flush the room data from room data queue
   *
   */
  void flush_room_data_queue() {
    if(keyframes.empty()) {
      return;
    } else if(room_data_queue.empty()) {
      // std::cout << "room data queue is empty" << std::endl;
      return;
    }

    for(const auto& room_data_msg : room_data_queue) {
      for(const auto& room_data : room_data_msg.rooms) {
        // float dist_robot_room = sqrt(pow(room_data.room_center.x - latest_keyframe->node->estimate().matrix()(0,3),2) + pow(room_data.room_center.y - latest_keyframe->node->estimate().matrix()(1,3),2));
        // std::cout << "dist robot room: " << dist_robot_room << std::endl;
        if(room_data.x_planes.size() == 2 && room_data.y_planes.size() == 2) {
          lookup_rooms(room_data);
        }
        // x infinite_room
        else if(room_data.x_planes.size() == 2 && room_data.y_planes.size() == 0) {
          lookup_corridors(plane_class::X_VERT_PLANE, room_data);
        }
        // y infinite_room
        else if(room_data.x_planes.size() == 0 && room_data.y_planes.size() == 2) {
          lookup_corridors(plane_class::Y_VERT_PLANE, room_data);
        }
      }

      room_data_queue_mutex.lock();
      room_data_queue.pop_front();
      room_data_queue_mutex.unlock();
    }
  }

  /**
   * @brief flush the accumulated cloud seg queue
   */
  bool flush_clouds_seg_queue() {
    std::lock_guard<std::mutex> lock(cloud_seg_mutex);

    if(keyframes.empty()) {
      std::cout << "No keyframes" << std::endl;
      return false;
    } else if(clouds_seg_queue.empty()) {
      std::cout << "Clouds seg queue is empty" << std::endl;
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& clouds_seg_msg : clouds_seg_queue) {
      std::vector<plane_data_list> x_det_corridor_candidates, y_det_corridor_candidates;
      std::vector<plane_data_list> x_det_room_candidates, y_det_room_candidates;

      for(const auto& cloud_seg_msg : clouds_seg_msg->pointclouds) {
        if(cloud_seg_msg.header.stamp > latest_keyframe_stamp) {
          // std::cout << "cloud_seg time is greater than last keyframe stamp" << std::endl;
          break;
        }

        auto found = keyframe_hash.find(cloud_seg_msg.header.stamp);
        if(found == keyframe_hash.end()) {
          continue;
        }

        pcl::PointCloud<PointNormal>::Ptr cloud_seg_body(new pcl::PointCloud<PointNormal>());
        pcl::fromROSMsg(cloud_seg_msg, *cloud_seg_body);

        if(cloud_seg_body->points.size() < min_plane_points) continue;

        const auto& keyframe = found->second;
        keyframe->cloud_seg_body = cloud_seg_body;

        g2o::Plane3D det_plane_body_frame = Eigen::Vector4d(cloud_seg_body->back().normal_x, cloud_seg_body->back().normal_y, cloud_seg_body->back().normal_z, cloud_seg_body->back().curvature);
        bool found_corridor_candidates = false;
        bool found_room_candidates = false;
        plane_data_list plane_id_pair;

        int plane_type = map_detected_planes(keyframe, det_plane_body_frame, found_corridor_candidates, found_room_candidates, plane_id_pair);
        switch(plane_type) {
          case plane_class::X_VERT_PLANE: {
            if(found_corridor_candidates) {
              x_det_corridor_candidates.push_back(plane_id_pair);
            }
            if(found_room_candidates) {
              x_det_room_candidates.push_back(plane_id_pair);
            }
            updated = true;
            break;
          }
          case plane_class::Y_VERT_PLANE: {
            if(found_corridor_candidates) {
              y_det_corridor_candidates.push_back(plane_id_pair);
            }
            if(found_room_candidates) {
              y_det_room_candidates.push_back(plane_id_pair);
            }
            updated = true;
            break;
          }
          case plane_class::HORT_PLANE: {
            updated = true;
            break;
          }
          default: {
            break;
          }
        }
      }

      if(use_corridor_constraint) {
        lookup_corridors(x_det_corridor_candidates, y_det_corridor_candidates);
      }

      if(use_room_constraint) {
        lookup_rooms(x_det_room_candidates, y_det_room_candidates);
      }
    }

    auto remove_loc = std::upper_bound(clouds_seg_queue.begin(), clouds_seg_queue.end(), latest_keyframe_stamp, [=](const ros::Time& stamp, const s_graphs::PointClouds::Ptr& clouds_seg) { return stamp < clouds_seg->header.stamp; });
    clouds_seg_queue.erase(clouds_seg_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief detected plane mapping
   *
   */
  int map_detected_planes(KeyFrame::Ptr keyframe, g2o::Plane3D det_plane_body_frame, bool& found_corridor, bool& found_room, plane_data_list& plane_id_pair) {
    int plane_id;
    int plane_type = -1;

    g2o::Plane3D det_plane_map_frame = plane_in_map_frame(keyframe, det_plane_body_frame);

    /* Get the plane type based on the largest value of the normal orientation x,y and z */
    if(fabs(det_plane_map_frame.coeffs()(0)) > fabs(det_plane_map_frame.coeffs()(1)) && fabs(det_plane_map_frame.coeffs()(0)) > fabs(det_plane_map_frame.coeffs()(2)))
      plane_type = plane_class::X_VERT_PLANE;
    else if(fabs(det_plane_map_frame.coeffs()(1)) > fabs(det_plane_map_frame.coeffs()(0)) && fabs(det_plane_map_frame.coeffs()(1)) > fabs(det_plane_map_frame.coeffs()(2)))
      plane_type = plane_class::Y_VERT_PLANE;
    else if(fabs(det_plane_map_frame.coeffs()(2)) > fabs(det_plane_map_frame.coeffs()(0)) && fabs(det_plane_map_frame.coeffs()(2)) > fabs(det_plane_map_frame.coeffs()(1)))
      plane_type = plane_class::HORT_PLANE;

    switch(plane_type) {
      case plane_class::X_VERT_PLANE: {
        plane_id = sort_planes(plane_type, keyframe, det_plane_map_frame, det_plane_body_frame);
        ROS_DEBUG_NAMED("xplane information", "det xplane map frame %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2), det_plane_map_frame.coeffs()(3));
        /* check for potential x corridor and room candidates */
        pcl::PointXY start_point, end_point;
        float length = plane_length(keyframe->cloud_seg_body, start_point, end_point, keyframe->node);
        ROS_DEBUG_NAMED("xplane information", "length x plane %f", length);
        Eigen::Vector4d plane_unflipped = det_plane_map_frame.coeffs();
        correct_plane_d(plane_type, plane_unflipped);

        // x_plane_id_pair.plane = det_plane_map_frame;
        plane_id_pair.plane_local = det_plane_body_frame;
        plane_id_pair.plane_unflipped = plane_unflipped;
        plane_id_pair.plane_length = length;
        if(start_point.y < end_point.y) {
          plane_id_pair.start_point = start_point;
          plane_id_pair.end_point = end_point;
        } else {
          plane_id_pair.start_point = end_point;
          plane_id_pair.end_point = start_point;
        }
        plane_id_pair.plane_id = plane_id;
        plane_id_pair.keyframe_node = keyframe->node;

        if(length >= corridor_min_plane_length) {
          ROS_DEBUG_NAMED("xplane information", "Added x plane as corridor");
          found_corridor = true;
        }
        if(length >= room_min_plane_length && length <= room_max_plane_length) {
          ROS_DEBUG_NAMED("xplane information", "Added x plane as room");
          found_room = true;
        }
        break;
      }
      case plane_class::Y_VERT_PLANE: {
        plane_id = sort_planes(plane_type, keyframe, det_plane_map_frame, det_plane_body_frame);
        ROS_DEBUG_NAMED("yplane information", "det yplane map frame %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2), det_plane_map_frame.coeffs()(3));

        /* check for potential y corridor and room candidates */
        pcl::PointXY start_point, end_point;
        float length = plane_length(keyframe->cloud_seg_body, start_point, end_point, keyframe->node);
        ROS_DEBUG_NAMED("yplane information", "length y plane %f", length);
        Eigen::Vector4d plane_unflipped = det_plane_map_frame.coeffs();
        correct_plane_d(plane_type, plane_unflipped);

        // y_plane_id_pair.plane = det_plane_map_frame;
        plane_id_pair.plane_local = det_plane_body_frame;
        plane_id_pair.plane_unflipped = plane_unflipped;
        if(start_point.x < end_point.x) {
          plane_id_pair.start_point = start_point;
          plane_id_pair.end_point = end_point;
        } else {
          plane_id_pair.start_point = end_point;
          plane_id_pair.end_point = start_point;
        }
        plane_id_pair.plane_length = length;
        plane_id_pair.plane_id = plane_id;
        plane_id_pair.keyframe_node = keyframe->node;

        if(length >= corridor_min_plane_length) {
          ROS_DEBUG_NAMED("yplane information", "Added y plane as corridor");
          found_corridor = true;
        }
        if(length >= room_min_plane_length && length <= room_max_plane_length) {
          ROS_DEBUG_NAMED("yplane information", "Added y plane as room");
          found_room = true;
        }
        break;
      }
      case plane_class::HORT_PLANE: {
        plane_id = sort_planes(plane_type, keyframe, det_plane_map_frame, det_plane_body_frame);
        break;
      }
      default:
        std::cout << "No planes found for mapping " << std::endl;
        break;
    }

    return plane_type;
  }

  /**
   * @brief sort and factor the detected planes
   *
   */
  int sort_planes(int plane_type, KeyFrame::Ptr keyframe, g2o::Plane3D det_plane_map_frame, g2o::Plane3D det_plane_body_frame) {
    int plane_id = factor_planes(keyframe, det_plane_map_frame, det_plane_body_frame, plane_type);

    return plane_id;
  }

  /**
   * @brief convert body plane coefficients to map frame
   */
  g2o::Plane3D plane_in_map_frame(KeyFrame::Ptr keyframe, g2o::Plane3D det_plane_body_frame) {
    g2o::Plane3D det_plane_map_frame;
    Eigen::Vector4d map_coeffs;

    Eigen::Isometry3d w2n = keyframe->node->estimate();
    map_coeffs.head<3>() = w2n.rotation() * det_plane_body_frame.coeffs().head<3>();
    map_coeffs(3) = det_plane_body_frame.coeffs()(3) - w2n.translation().dot(map_coeffs.head<3>());
    det_plane_map_frame = map_coeffs;

    return det_plane_map_frame;
  }

  /**
   * @brief create vertical plane factors
   */
  int factor_planes(KeyFrame::Ptr keyframe, g2o::Plane3D det_plane_map_frame, g2o::Plane3D det_plane_body_frame, int plane_type) {
    g2o::VertexPlane* plane_node;

    Eigen::Matrix4d Gij;
    Gij.setZero();
    if(use_point_to_plane) {
      auto it = keyframe->cloud_seg_body->points.begin();
      while(it != keyframe->cloud_seg_body->points.end()) {
        PointNormal point_tmp;
        point_tmp = *it;
        Eigen::Vector4d point(point_tmp.x, point_tmp.y, point_tmp.z, 1);
        double point_to_plane_d = det_plane_map_frame.coeffs().transpose() * keyframe->node->estimate().matrix() * point;

        if(abs(point_to_plane_d) < 0.1) {
          Gij += point * point.transpose();
          ++it;
        } else {
          it = keyframe->cloud_seg_body->points.erase(it);
        }
      }
    }

    std::pair<int, int> data_association;
    data_association.first = -1;
    data_association = associate_plane(keyframe, det_plane_body_frame.coeffs(), keyframe->cloud_seg_body, plane_type);

    switch(plane_type) {
      case plane_class::X_VERT_PLANE: {
        if(x_vert_planes.empty() || data_association.first == -1) {
          data_association.first = graph_slam->num_vertices_local();
          plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
          VerticalPlanes vert_plane;
          vert_plane.id = data_association.first;
          vert_plane.plane = det_plane_map_frame.coeffs();
          vert_plane.cloud_seg_body = keyframe->cloud_seg_body;
          vert_plane.cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          vert_plane.keyframe_node_vec.push_back(keyframe->node);
          vert_plane.keyframe_node = keyframe->node;
          vert_plane.plane_node = plane_node;
          vert_plane.cloud_seg_map = nullptr;
          vert_plane.covariance = Eigen::Matrix3d::Identity();
          x_vert_planes.push_back(vert_plane);
          keyframe->x_plane_ids.push_back(vert_plane.id);
          ROS_DEBUG_NAMED("xplane association", "Added new x vertical plane node with coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2), det_plane_map_frame.coeffs()(3));
        } else {
          plane_node = x_vert_planes[data_association.second].plane_node;
          x_vert_planes[data_association.second].cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          x_vert_planes[data_association.second].keyframe_node_vec.push_back(keyframe->node);
          keyframe->x_plane_ids.push_back(x_vert_planes[data_association.second].id);

          ROS_DEBUG_NAMED("xplane association", "matched x vert plane with coeffs %f %f %f %f to mapped x vert plane of coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2),
                          det_plane_map_frame.coeffs()(3), x_vert_planes[data_association.second].plane_node->estimate().coeffs()(0), x_vert_planes[data_association.second].plane_node->estimate().coeffs()(1),
                          x_vert_planes[data_association.second].plane_node->estimate().coeffs()(2), x_vert_planes[data_association.second].plane_node->estimate().coeffs()(3));
        }
        break;
      }
      case plane_class::Y_VERT_PLANE: {
        if(y_vert_planes.empty() || data_association.first == -1) {
          data_association.first = graph_slam->num_vertices_local();
          plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
          VerticalPlanes vert_plane;
          vert_plane.id = data_association.first;
          vert_plane.plane = det_plane_map_frame.coeffs();
          vert_plane.cloud_seg_body = keyframe->cloud_seg_body;
          vert_plane.cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          vert_plane.keyframe_node_vec.push_back(keyframe->node);
          vert_plane.keyframe_node = keyframe->node;
          vert_plane.plane_node = plane_node;
          vert_plane.cloud_seg_map = nullptr;
          vert_plane.covariance = Eigen::Matrix3d::Identity();
          y_vert_planes.push_back(vert_plane);
          keyframe->y_plane_ids.push_back(vert_plane.id);

          ROS_DEBUG_NAMED("yplane association", "Added new y vertical plane node with coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2), det_plane_map_frame.coeffs()(3));
        } else {
          plane_node = y_vert_planes[data_association.second].plane_node;
          y_vert_planes[data_association.second].cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          y_vert_planes[data_association.second].keyframe_node_vec.push_back(keyframe->node);
          ROS_DEBUG_NAMED("yplane association", "matched y vert plane with coeffs %f %f %f %f to mapped y vert plane of coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2),
                          det_plane_map_frame.coeffs()(3), y_vert_planes[data_association.second].plane_node->estimate().coeffs()(0), y_vert_planes[data_association.second].plane_node->estimate().coeffs()(1),
                          y_vert_planes[data_association.second].plane_node->estimate().coeffs()(2), y_vert_planes[data_association.second].plane_node->estimate().coeffs()(3));
          keyframe->y_plane_ids.push_back(y_vert_planes[data_association.second].id);
        }
        break;
      }
      case plane_class::HORT_PLANE: {
        if(hort_planes.empty() || data_association.first == -1) {
          data_association.first = graph_slam->num_vertices_local();
          plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
          HorizontalPlanes hort_plane;
          hort_plane.id = data_association.first;
          hort_plane.plane = det_plane_map_frame.coeffs();
          hort_plane.cloud_seg_body = keyframe->cloud_seg_body;
          hort_plane.cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          hort_plane.keyframe_node_vec.push_back(keyframe->node);
          hort_plane.keyframe_node = keyframe->node;
          hort_plane.plane_node = plane_node;
          hort_plane.cloud_seg_map = nullptr;
          hort_plane.covariance = Eigen::Matrix3d::Identity();
          hort_planes.push_back(hort_plane);
          keyframe->hort_plane_ids.push_back(hort_plane.id);

          ROS_DEBUG_NAMED("hort plane association", "Added new horizontal plane node with coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2), det_plane_map_frame.coeffs()(3));
        } else {
          plane_node = hort_planes[data_association.second].plane_node;
          hort_planes[data_association.second].cloud_seg_body_vec.push_back(keyframe->cloud_seg_body);
          hort_planes[data_association.second].keyframe_node_vec.push_back(keyframe->node);
          ROS_DEBUG_NAMED("hort plane association", "matched hort plane with coeffs %f %f %f %f to mapped hort plane of coeffs %f %f %f %f", det_plane_map_frame.coeffs()(0), det_plane_map_frame.coeffs()(1), det_plane_map_frame.coeffs()(2),
                          det_plane_map_frame.coeffs()(3), hort_planes[data_association.second].plane_node->estimate().coeffs()(0), hort_planes[data_association.second].plane_node->estimate().coeffs()(1),
                          hort_planes[data_association.second].plane_node->estimate().coeffs()(2), hort_planes[data_association.second].plane_node->estimate().coeffs()(3));
          keyframe->hort_plane_ids.push_back(hort_planes[data_association.second].id);
        }
        break;
      }
      default:
        std::cout << "factoring planes function had a weird error " << std::endl;
        break;
    }

    if(use_point_to_plane) {
      Eigen::Matrix<double, 1, 1> information(0.001);
      auto edge = graph_slam->add_se3_point_to_plane_edge(keyframe->node, plane_node, Gij, information);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    } else {
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, plane_node, det_plane_body_frame.coeffs(), information);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
    }

    convert_plane_points_to_map();

    return data_association.first;
  }

  /**
   * @brief data assoction betweeen the planes
   */
  std::pair<int, int> associate_plane(KeyFrame::Ptr keyframe, g2o::Plane3D det_plane, pcl::PointCloud<PointNormal>::Ptr cloud_seg_body, int plane_type) {
    std::pair<int, int> data_association;
    double vert_min_maha_dist = 100;
    double hort_min_maha_dist = 100;
    Eigen::Isometry3d m2n = keyframe->estimate().inverse();

    switch(plane_type) {
      case plane_class::X_VERT_PLANE: {
        for(int i = 0; i < x_vert_planes.size(); ++i) {
          g2o::Plane3D local_plane = m2n * x_vert_planes[i].plane;
          Eigen::Vector3d error = local_plane.ominus(det_plane);
          double maha_dist = sqrt(error.transpose() * x_vert_planes[i].covariance.inverse() * error);
          ROS_DEBUG_NAMED("xplane plane association", "maha distance xplane: %f", maha_dist);

          // printf("\n maha distance x: %f", maha_dist);

          if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);
          }
          if(maha_dist < vert_min_maha_dist) {
            vert_min_maha_dist = maha_dist;
            data_association.first = x_vert_planes[i].id;
            data_association.second = i;
          }
        }
        if(vert_min_maha_dist < plane_dist_threshold) {
          float min_segment = std::numeric_limits<float>::max();
          pcl::PointCloud<PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<PointNormal>());
          Eigen::Matrix4f current_keyframe_pose = keyframe->estimate().matrix().cast<float>();
          for(size_t j = 0; j < cloud_seg_body->points.size(); ++j) {
            PointNormal dst_pt;
            dst_pt.getVector4fMap() = current_keyframe_pose * cloud_seg_body->points[j].getVector4fMap();
            cloud_seg_detected->points.push_back(dst_pt);
          }
          min_segment = get_min_segment(x_vert_planes[data_association.second].cloud_seg_map, cloud_seg_detected);
          // std::cout << "X plane min maha distance: " << vert_min_maha_dist << std::endl;
          // std::cout << "X plane min segment: " << min_segment << std::endl;
          if(min_segment > 0.5) {
            data_association.first = -1;
          }
        } else
          data_association.first = -1;
        break;
      }
      case plane_class::Y_VERT_PLANE: {
        for(int i = 0; i < y_vert_planes.size(); ++i) {
          float dist = fabs(det_plane.coeffs()(3) - y_vert_planes[i].plane.coeffs()(3));
          g2o::Plane3D local_plane = m2n * y_vert_planes[i].plane;
          Eigen::Vector3d error = local_plane.ominus(det_plane);
          double maha_dist = sqrt(error.transpose() * y_vert_planes[i].covariance.inverse() * error);
          ROS_DEBUG_NAMED("yplane plane association", "maha distance yplane: %f", maha_dist);
          // printf("\n maha distance y: %f", maha_dist);

          if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);
          }
          if(maha_dist < vert_min_maha_dist) {
            vert_min_maha_dist = maha_dist;
            data_association.first = y_vert_planes[i].id;
            data_association.second = i;
          }
        }
        if(vert_min_maha_dist < plane_dist_threshold) {
          float min_segment = std::numeric_limits<float>::max();
          pcl::PointCloud<PointNormal>::Ptr cloud_seg_detected(new pcl::PointCloud<PointNormal>());
          Eigen::Matrix4f current_keyframe_pose = keyframe->estimate().matrix().cast<float>();
          for(size_t j = 0; j < cloud_seg_body->points.size(); ++j) {
            PointNormal dst_pt;
            dst_pt.getVector4fMap() = current_keyframe_pose * cloud_seg_body->points[j].getVector4fMap();
            cloud_seg_detected->points.push_back(dst_pt);
          }
          min_segment = get_min_segment(y_vert_planes[data_association.second].cloud_seg_map, cloud_seg_detected);
          // std::cout << "mapped plane id: " << data_association.first << std::endl;
          // std::cout << "Y plane min maha distance: " << vert_min_maha_dist << std::endl;
          // std::cout << "Y plane min segment: " << min_segment << std::endl;
          // std::cout << "Y plane coeffs: " << y_vert_planes[data_association.second].plane.coeffs() << std::endl;
          if(min_segment > 0.5) {
            data_association.first = -1;
          }
        } else
          data_association.first = -1;
        break;
      }
      case plane_class::HORT_PLANE: {
        for(int i = 0; i < hort_planes.size(); ++i) {
          g2o::Plane3D local_plane = m2n * hort_planes[i].plane;
          Eigen::Vector3d error = local_plane.ominus(det_plane);
          double maha_dist = sqrt(error.transpose() * hort_planes[i].covariance.inverse() * error);
          // std::cout << "cov hor: " << hort_planes[i].covariance.inverse() << std::endl;
          ROS_DEBUG_NAMED("hort plane association", "maha distance hort: %f", maha_dist);

          if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);
          }
          if(maha_dist < hort_min_maha_dist) {
            vert_min_maha_dist = maha_dist;
            data_association.first = hort_planes[i].id;
            data_association.second = i;
          }
        }

        if(hort_min_maha_dist > plane_dist_threshold) data_association.first = -1;

        break;
      }
      default:
        std::cout << "associating planes had an error " << std::endl;
        break;
    }

    // printf("\n vert min maha dist: %f", vert_min_maha_dist);
    // printf("\n hort min maha dist: %f", hort_min_maha_dist);

    return data_association;
  }

  void lookup_corridors(std::vector<plane_data_list> x_det_corridor_candidates, std::vector<plane_data_list> y_det_corridor_candidates) {
    std::vector<structure_data_list> x_corridor = sort_corridors(plane_class::X_VERT_PLANE, x_det_corridor_candidates);
    std::vector<structure_data_list> y_corridor = sort_corridors(plane_class::Y_VERT_PLANE, y_det_corridor_candidates);

    std::vector<plane_data_list> x_corridor_refined = refine_corridors(x_corridor);
    if(x_corridor_refined.size() == 2) factor_corridors(plane_class::X_VERT_PLANE, x_corridor_refined[0], x_corridor_refined[1]);

    std::vector<plane_data_list> y_corridor_refined = refine_corridors(y_corridor);
    if(y_corridor_refined.size() == 2) factor_corridors(plane_class::Y_VERT_PLANE, y_corridor_refined[0], y_corridor_refined[1]);
  }

  void lookup_corridors(const int& plane_type, const s_graphs::RoomData room_data) {
    if(plane_type == plane_class::X_VERT_PLANE) {
      // check the distance with the current room vector

      // factor the infinite_room here
      std::cout << "factoring x corridor" << std::endl;
      Eigen::Vector4d x_plane1(room_data.x_planes[0].nx, room_data.x_planes[0].ny, room_data.x_planes[0].nz, room_data.x_planes[0].d);
      Eigen::Vector4d x_plane2(room_data.x_planes[1].nx, room_data.x_planes[1].ny, room_data.x_planes[1].nz, room_data.x_planes[1].d);
      plane_data_list x_plane1_data, x_plane2_data;
      x_plane1_data.plane_id = room_data.x_planes[0].id;
      x_plane1_data.plane_unflipped = x_plane1;
      x_plane1_data.plane_centroid(0) = room_data.room_center.x;
      x_plane1_data.plane_centroid(1) = room_data.room_center.y;

      x_plane2_data.plane_id = room_data.x_planes[1].id;
      x_plane2_data.plane_unflipped = x_plane2;
      x_plane2_data.plane_centroid(0) = room_data.room_center.x;
      x_plane2_data.plane_centroid(1) = room_data.room_center.y;

      factor_corridors(plane_class::X_VERT_PLANE, x_plane1_data, x_plane2_data);
    }

    else if(plane_type == plane_class::Y_VERT_PLANE) {
      // factor the infinite_room here
      std::cout << "factoring y corridors" << std::endl;
      Eigen::Vector4d y_plane1(room_data.y_planes[0].nx, room_data.y_planes[0].ny, room_data.y_planes[0].nz, room_data.y_planes[0].d);
      Eigen::Vector4d y_plane2(room_data.y_planes[1].nx, room_data.y_planes[1].ny, room_data.y_planes[1].nz, room_data.y_planes[1].d);
      plane_data_list y_plane1_data, y_plane2_data;
      y_plane1_data.plane_id = room_data.y_planes[0].id;
      y_plane1_data.plane_unflipped = y_plane1;
      y_plane1_data.plane_centroid(0) = room_data.room_center.x;
      y_plane1_data.plane_centroid(1) = room_data.room_center.y;

      y_plane2_data.plane_id = room_data.y_planes[1].id;
      y_plane2_data.plane_unflipped = y_plane2;
      y_plane2_data.plane_centroid(0) = room_data.room_center.x;
      y_plane2_data.plane_centroid(1) = room_data.room_center.y;

      factor_corridors(plane_class::Y_VERT_PLANE, y_plane1_data, y_plane2_data);
    }
  }

  void lookup_rooms(std::vector<plane_data_list> x_det_room_candidates, std::vector<plane_data_list> y_det_room_candidates) {
    std::vector<structure_data_list> x_room_pair_vec = sort_rooms(plane_class::X_VERT_PLANE, x_det_room_candidates);
    std::vector<structure_data_list> y_room_pair_vec = sort_rooms(plane_class::Y_VERT_PLANE, y_det_room_candidates);
    std::pair<std::vector<plane_data_list>, std::vector<plane_data_list>> refined_room_pair = refine_rooms(x_room_pair_vec, y_room_pair_vec);

    if(refined_room_pair.first.size() == 2 && refined_room_pair.second.size() == 2) {
      factor_rooms(refined_room_pair.first, refined_room_pair.second);
    }
  }

  void lookup_rooms(const s_graphs::RoomData room_data) {
    // float min_dist_room_x_corr = 100;
    // Corridors matched_x_corridor;
    // for(const auto& current_x_corridor : x_corridors) {
    //   if((room_data.x_planes[0].id == current_x_corridor.plane1_id || room_data.x_planes[0].id == current_x_corridor.plane2_id) && (room_data.x_planes[1].id == current_x_corridor.plane1_id || room_data.x_planes[1].id == current_x_corridor.plane2_id)) {
    //     min_dist_room_x_corr = 0;
    //     matched_x_corridor = current_x_corridor;
    //     break;
    //   }
    //   float dist_room_x_corr = sqrt(pow(room_data.room_center.x - current_x_corridor.node->estimate()(0), 2) + pow(room_data.room_center.y - current_x_corridor.node->estimate()(1), 2));
    //   if(dist_room_x_corr < min_dist_room_x_corr) {
    //     min_dist_room_x_corr = dist_room_x_corr;
    //     matched_x_corridor = current_x_corridor;
    //   }
    // }

    // float min_dist_room_y_corr = 100;
    // Corridors matched_y_corridor;
    // for(const auto& current_y_corridor : y_corridors) {
    //   if((room_data.y_planes[0].id == current_y_corridor.plane1_id || room_data.y_planes[0].id == current_y_corridor.plane2_id) && (room_data.y_planes[1].id == current_y_corridor.plane1_id || room_data.y_planes[1].id == current_y_corridor.plane2_id)) {
    //     min_dist_room_y_corr = 0;
    //     matched_y_corridor = current_y_corridor;
    //     break;
    //   }

    //   float dist_room_y_corr = sqrt(pow(room_data.room_center.x - current_y_corridor.node->estimate()(0), 2) + pow(room_data.room_center.y - current_y_corridor.node->estimate()(1), 2));
    //   if(dist_room_y_corr < min_dist_room_y_corr) {
    //     min_dist_room_y_corr = dist_room_y_corr;
    //     matched_y_corridor = current_y_corridor;
    //   }
    // }

    // auto found_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[0].id);
    // auto found_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.x_planes[1].id);
    // auto found_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[0].id);
    // auto found_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == room_data.y_planes[1].id);

    // if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr < 1.0) {
    //   std::cout << "Adding a room using mapped x and y corridor planes " << std::endl;
    //   map_room_from_existing_corridors(room_data, matched_x_corridor, matched_y_corridor, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    //   remove_mapped_corridor(plane_class::X_VERT_PLANE, matched_x_corridor);
    //   remove_mapped_corridor(plane_class::Y_VERT_PLANE, matched_y_corridor);
    // } else if(min_dist_room_x_corr < 1.0 && min_dist_room_y_corr > 1.0) {
    //   map_room_from_existing_x_corridor(room_data, matched_x_corridor, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    //   std::cout << "Will add room using mapped x corridor planes " << std::endl;
    //   remove_mapped_corridor(plane_class::X_VERT_PLANE, matched_x_corridor);
    // } else if(min_dist_room_y_corr < 1.0 && min_dist_room_x_corr > 1.0) {
    //   std::cout << "Will add room using mapped y corridor planes " << std::endl;
    //   map_room_from_existing_y_corridor(room_data, matched_y_corridor, (*found_x_plane1), (*found_x_plane2), (*found_y_plane1), (*found_y_plane1));
    //   remove_mapped_corridor(plane_class::Y_VERT_PLANE, matched_y_corridor);
    // }

    Eigen::Vector4d x_plane1(room_data.x_planes[0].nx, room_data.x_planes[0].ny, room_data.x_planes[0].nz, room_data.x_planes[0].d);
    Eigen::Vector4d x_plane2(room_data.x_planes[1].nx, room_data.x_planes[1].ny, room_data.x_planes[1].nz, room_data.x_planes[1].d);
    Eigen::Vector4d y_plane1(room_data.y_planes[0].nx, room_data.y_planes[0].ny, room_data.y_planes[0].nz, room_data.y_planes[0].d);
    Eigen::Vector4d y_plane2(room_data.y_planes[1].nx, room_data.y_planes[1].ny, room_data.y_planes[1].nz, room_data.y_planes[1].d);

    plane_data_list x_plane1_data, x_plane2_data;
    plane_data_list y_plane1_data, y_plane2_data;

    x_plane1_data.plane_id = room_data.x_planes[0].id;
    x_plane1_data.plane_unflipped = x_plane1;
    x_plane1_data.plane_centroid(0) = room_data.room_center.x;
    x_plane1_data.plane_centroid(1) = room_data.room_center.y;
    x_plane2_data.plane_id = room_data.x_planes[1].id;
    x_plane2_data.plane_unflipped = x_plane2;

    y_plane1_data.plane_id = room_data.y_planes[0].id;
    y_plane1_data.plane_unflipped = y_plane1;

    y_plane2_data.plane_id = room_data.y_planes[1].id;
    y_plane2_data.plane_unflipped = y_plane2;

    std::vector<plane_data_list> x_planes_room, y_planes_room;
    x_planes_room.push_back(x_plane1_data);
    x_planes_room.push_back(x_plane2_data);
    y_planes_room.push_back(y_plane1_data);
    y_planes_room.push_back(y_plane2_data);

    factor_rooms(x_planes_room, y_planes_room);
  }

  // void map_room_from_existing_corridors(const s_graphs::RoomData& det_room_data, const Corridors& matched_x_corridor, const Corridors& matched_y_corridor, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  //   g2o::VertexRoomXYLB* room_node;
  //   std::pair<int, int> room_data_association;

  //   std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  //   Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  //   room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  //   if((rooms_vec.empty() || room_data_association.first == -1)) {
  //     std::cout << "Add a room using mapped x and y corridors at pose" << room_pose << std::endl;
  //     room_data_association.first = graph_slam->num_vertices_local();
  //     room_node = graph_slam->add_room_node(room_pose);
  //     Rooms det_room;
  //     det_room.id = room_data_association.first;
  //     det_room.plane_x1 = matched_x_corridor.plane1;
  //     det_room.plane_x2 = matched_x_corridor.plane2;
  //     det_room.plane_y1 = matched_y_corridor.plane1;
  //     det_room.plane_y2 = matched_y_corridor.plane2;
  //     det_room.plane_x1_id = matched_x_corridor.plane1_id;
  //     det_room.plane_x2_id = matched_x_corridor.plane2_id;
  //     det_room.plane_y1_id = matched_y_corridor.plane1_id;
  //     det_room.plane_y2_id = matched_y_corridor.plane2_id;
  //     det_room.node = room_node;
  //     rooms_vec.push_back(det_room);
  //     return;
  //   } else
  //     return;
  // }

  void remove_mapped_corridor(const int plane_type, s_graphs::Corridors matched_corridor) {
    std::set<g2o::HyperGraph::Edge*> edges = matched_corridor.node->edges();
    for(auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
      // g2o::EdgeRoom2Planes* edge_room_2planes = dynamic_cast<g2o::EdgeRoom2Planes*>(*edge_itr);
      // if(edge_room_2planes) {
      //   if(graph_slam->remove_room_2planes_edge(edge_room_2planes)) std::cout << "removed edge - room-2planes " << std::endl;
      //   continue;
      // }
    }

    // if(plane_type == plane_class::X_VERT_PLANE) {
    //   if(graph_slam->remove_room_node(matched_corridor.node)) {
    //     auto mapped_corridor = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::id, _1) == matched_corridor.id);
    //     x_corridors.erase(mapped_corridor);
    //     std::cout << "removed overlapped x-corridor " << std::endl;
    //   }
    // } else if(plane_type == plane_class::Y_VERT_PLANE) {
    //   if(graph_slam->remove_room_node(matched_corridor.node)) {
    //     auto mapped_corridor = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::id, _1) == matched_corridor.id);
    //     y_corridors.erase(mapped_corridor);
    //     std::cout << "removed overlapped y-corridor " << std::endl;
    //   }
    // }
  }

  // void map_room_from_existing_x_corridor(const s_graphs::RoomData& det_room_data, const s_graphs::Corridors& matched_x_corridor, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  //   g2o::VertexRoomXYLB* room_node;
  //   std::pair<int, int> room_data_association;

  //   std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  //   Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  //   room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  //   if((rooms_vec.empty() || room_data_association.first == -1)) {
  //     std::cout << "Add a room using mapped x corridors planes at pose" << room_pose << std::endl;
  //     room_data_association.first = graph_slam->num_vertices_local();
  //     room_node = graph_slam->add_room_node(room_pose);
  //     Rooms det_room;
  //     det_room.id = room_data_association.first;
  //     det_room.plane_x1 = matched_x_corridor.plane1;
  //     det_room.plane_x2 = matched_x_corridor.plane2;
  //     Eigen::Vector4d y_plane1(det_room_data.y_planes[0].nx, det_room_data.y_planes[0].ny, det_room_data.y_planes[0].nz, det_room_data.y_planes[0].d);
  //     Eigen::Vector4d y_plane2(det_room_data.y_planes[1].nx, det_room_data.y_planes[1].ny, det_room_data.y_planes[1].nz, det_room_data.y_planes[1].d);
  //     det_room.plane_y1 = y_plane1;
  //     det_room.plane_y2 = y_plane2;
  //     det_room.plane_x1_id = matched_x_corridor.plane1_id;
  //     det_room.plane_x2_id = matched_x_corridor.plane2_id;
  //     det_room.plane_y1_id = det_room_data.y_planes[0].id;
  //     det_room.plane_y2_id = det_room_data.y_planes[1].id;
  //     det_room.node = room_node;
  //     rooms_vec.push_back(det_room);
  //     return;
  //   } else
  //     return;
  // }

  // void map_room_from_existing_y_corridor(const s_graphs::RoomData& det_room_data, const Corridors& matched_y_corridor, const VerticalPlanes& x_plane1, const VerticalPlanes& x_plane2, const VerticalPlanes& y_plane1, const VerticalPlanes& y_plane2) {
  //   g2o::VertexRoomXYLB* room_node;
  //   std::pair<int, int> room_data_association;

  //   std::vector<std::pair<VerticalPlanes, VerticalPlanes>> detected_mapped_plane_pairs;
  //   Eigen::Vector2d room_pose(det_room_data.room_center.x, det_room_data.room_center.y);
  //   room_data_association = associate_rooms(room_pose, rooms_vec, x_vert_planes, y_vert_planes, x_plane1, x_plane2, y_plane1, y_plane2, detected_mapped_plane_pairs);
  //   if((rooms_vec.empty() || room_data_association.first == -1)) {
  //     std::cout << "Add a room using mapped y corridors planes at pose" << room_pose << std::endl;
  //     room_data_association.first = graph_slam->num_vertices_local();
  //     room_node = graph_slam->add_room_node(room_pose);
  //     Rooms det_room;
  //     det_room.id = room_data_association.first;
  //     Eigen::Vector4d x_plane1(det_room_data.x_planes[0].nx, det_room_data.x_planes[0].ny, det_room_data.x_planes[0].nz, det_room_data.x_planes[0].d);
  //     Eigen::Vector4d x_plane2(det_room_data.x_planes[1].nx, det_room_data.x_planes[1].ny, det_room_data.x_planes[1].nz, det_room_data.x_planes[1].d);
  //     det_room.plane_x1 = x_plane1;
  //     det_room.plane_x2 = x_plane2;
  //     det_room.plane_y1 = matched_y_corridor.plane1;
  //     det_room.plane_y2 = matched_y_corridor.plane2;
  //     det_room.plane_x1_id = det_room_data.x_planes[0].id;
  //     det_room.plane_x2_id = det_room_data.x_planes[1].id;
  //     det_room.plane_y1_id = matched_y_corridor.plane1_id;
  //     det_room.plane_y2_id = matched_y_corridor.plane2_id;
  //     det_room.node = room_node;
  //     rooms_vec.push_back(det_room);
  //     return;
  //   } else
  //     return;
  // }

  /**
   * @brief sort corridors and add their possible candidates for refinement
   */
  std::vector<structure_data_list> sort_corridors(int plane_type, std::vector<plane_data_list> corridor_candidates) {
    std::vector<structure_data_list> corridor_pair_vec;

    for(int i = 0; i < corridor_candidates.size(); ++i) {
      for(int j = i + 1; j < corridor_candidates.size(); ++j) {
        float corr_width = width_between_planes(corridor_candidates[i].plane_unflipped.coeffs(), corridor_candidates[j].plane_unflipped.coeffs());
        float diff_plane_length = fabs(corridor_candidates[i].plane_length - corridor_candidates[j].plane_length);
        float start_point_diff = point_difference(plane_type, corridor_candidates[i].start_point, corridor_candidates[j].start_point);
        float end_point_diff = point_difference(plane_type, corridor_candidates[i].end_point, corridor_candidates[j].end_point);
        float avg_plane_point_diff = (start_point_diff + end_point_diff) / 2;
        ROS_DEBUG_NAMED("corridor planes", "corr plane i coeffs %f %f %f %f", corridor_candidates[i].plane_unflipped.coeffs()(0), corridor_candidates[i].plane_unflipped.coeffs()(1), corridor_candidates[i].plane_unflipped.coeffs()(2),
                        corridor_candidates[i].plane_unflipped.coeffs()(3));
        ROS_DEBUG_NAMED("corridor planes", "corr plane j coeffs %f %f %f %f", corridor_candidates[j].plane_unflipped.coeffs()(0), corridor_candidates[j].plane_unflipped.coeffs()(1), corridor_candidates[j].plane_unflipped.coeffs()(2),
                        corridor_candidates[j].plane_unflipped.coeffs()(3));
        ROS_DEBUG_NAMED("corridor planes", "corr width %f", corr_width);
        ROS_DEBUG_NAMED("corridor planes", "plane length diff %f", diff_plane_length);
        ROS_DEBUG_NAMED("corridor planes", "avg plane point diff %f", avg_plane_point_diff);

        if(corridor_candidates[i].plane_unflipped.coeffs().head(3).dot(corridor_candidates[j].plane_unflipped.coeffs().head(3)) < 0 && (corr_width < corridor_max_width && corr_width > corridor_min_width) && diff_plane_length < corridor_plane_length_diff_threshold) {
          if(avg_plane_point_diff < corridor_point_diff_threshold) {
            structure_data_list corridor_pair;
            corridor_pair.plane1 = corridor_candidates[i];
            corridor_pair.plane2 = corridor_candidates[j];
            corridor_pair.width = corr_width;
            corridor_pair.length_diff = diff_plane_length;
            corridor_pair.avg_point_diff = avg_plane_point_diff;
            corridor_pair_vec.push_back(corridor_pair);
            ROS_DEBUG_NAMED("corridor planes", "adding corridor candidates");
          }
        }
      }
    }

    return corridor_pair_vec;
  }

  /**
   * @brief refine the sorted corridors
   */
  std::vector<plane_data_list> refine_corridors(std::vector<structure_data_list> corr_vec) {
    float min_corridor_diff = corridor_point_diff_threshold;
    std::vector<plane_data_list> corr_refined;
    corr_refined.resize(2);

    for(int i = 0; i < corr_vec.size(); ++i) {
      float corridor_diff = corr_vec[i].avg_point_diff;
      if(corridor_diff < min_corridor_diff) {
        min_corridor_diff = corridor_diff;
        corr_refined[0] = corr_vec[i].plane1;
        corr_refined[1] = corr_vec[i].plane2;
      }
    }

    if(min_corridor_diff >= corridor_point_diff_threshold) {
      std::vector<plane_data_list> corr_empty;
      corr_empty.resize(0);
      return corr_empty;
    } else
      return corr_refined;
  }

  /**
   * @brief sort the rooms candidates
   */
  std::vector<structure_data_list> sort_rooms(int plane_type, std::vector<plane_data_list> room_candidates) {
    std::vector<structure_data_list> room_pair_vec;

    for(int i = 0; i < room_candidates.size(); ++i) {
      for(int j = i + 1; j < room_candidates.size(); ++j) {
        float room_width = width_between_planes(room_candidates[i].plane_unflipped.coeffs(), room_candidates[j].plane_unflipped.coeffs());
        float diff_plane_length = fabs(room_candidates[i].plane_length - room_candidates[j].plane_length);
        float start_point_diff = point_difference(plane_type, room_candidates[i].start_point, room_candidates[j].start_point);
        float end_point_diff = point_difference(plane_type, room_candidates[i].end_point, room_candidates[j].end_point);
        float avg_plane_point_diff = (start_point_diff + end_point_diff) / 2;
        ROS_DEBUG_NAMED("room planes", "room plane i coeffs %f %f %f %f", room_candidates[i].plane_unflipped.coeffs()(0), room_candidates[i].plane_unflipped.coeffs()(1), room_candidates[i].plane_unflipped.coeffs()(2), room_candidates[i].plane_unflipped.coeffs()(3));
        ROS_DEBUG_NAMED("room planes", "room plane j coeffs %f %f %f %f", room_candidates[j].plane_unflipped.coeffs()(0), room_candidates[j].plane_unflipped.coeffs()(1), room_candidates[j].plane_unflipped.coeffs()(2), room_candidates[j].plane_unflipped.coeffs()(3));
        ROS_DEBUG_NAMED("room planes", "room width %f", room_width);
        ROS_DEBUG_NAMED("room planes", "room plane lenght diff %f", diff_plane_length);
        ROS_DEBUG_NAMED("room planes", "room plane point diff %f", avg_plane_point_diff);

        if(room_candidates[i].plane_unflipped.coeffs().head(3).dot(room_candidates[j].plane_unflipped.coeffs().head(3)) < 0 && (room_width > room_min_width && room_width < room_max_width) && diff_plane_length < room_plane_length_diff_threshold) {
          if(avg_plane_point_diff < room_point_diff_threshold) {
            structure_data_list room_pair;
            room_pair.plane1 = room_candidates[i];
            room_pair.plane2 = room_candidates[j];
            room_pair.width = room_width;
            room_pair.length_diff = diff_plane_length;
            room_pair.avg_point_diff = avg_plane_point_diff;
            room_pair_vec.push_back(room_pair);
            ROS_DEBUG_NAMED("room planes", "adding room candidates");
          }
        }
      }
    }
    return room_pair_vec;
  }

  /**
   * @brief refine the sorted room candidates
   */
  std::pair<std::vector<plane_data_list>, std::vector<plane_data_list>> refine_rooms(std::vector<structure_data_list> x_room_vec, std::vector<structure_data_list> y_room_vec) {
    float min_room_point_diff = room_point_diff_threshold;
    std::vector<plane_data_list> x_room, y_room;
    x_room.resize(2);
    y_room.resize(2);

    for(int i = 0; i < x_room_vec.size(); ++i) {
      for(int j = 0; j < y_room_vec.size(); ++j) {
        float width_diff = fabs(x_room_vec[i].width - y_room_vec[j].width);
        if(width_diff < room_width_diff_threshold) {
          float room_diff = (x_room_vec[i].avg_point_diff + y_room_vec[j].avg_point_diff) / 2;
          if(room_diff < min_room_point_diff) {
            min_room_point_diff = room_diff;
            x_room[0] = x_room_vec[i].plane1;
            x_room[1] = x_room_vec[i].plane2;
            y_room[0] = y_room_vec[j].plane1;
            y_room[1] = y_room_vec[j].plane2;
          }
        }
      }
    }

    if(min_room_point_diff >= room_point_diff_threshold) {
      std::vector<plane_data_list> x_room_empty, y_room_empty;
      x_room_empty.resize(0);
      y_room_empty.resize(0);
      return std::make_pair(x_room_empty, x_room_empty);
    } else
      return std::make_pair(x_room, y_room);
  }

  /**
   * @brief this method creates the corridor vertex and adds edges between the vertex the detected planes
   */
  void factor_corridors(int plane_type, plane_data_list corr_plane1_pair, plane_data_list corr_plane2_pair) {
    g2o::VertexCorridor* corr_node;
    std::pair<int, int> corr_data_association;
    double meas_plane1, meas_plane2;
    Eigen::Matrix<double, 1, 1> information_corridor_plane(corridor_information);
    double corr_pose = compute_corridor_pose(plane_type, corr_plane1_pair.plane_unflipped.coeffs(), corr_plane2_pair.plane_unflipped.coeffs());
    // double corr_pose_local = corridor_pose_local(corr_plane1_pair.keyframe_node, corr_pose);
    ROS_DEBUG_NAMED("corridor planes", "final corridor plane 1 %f %f %f %f", corr_plane1_pair.plane_unflipped.coeffs()(0), corr_plane1_pair.plane_unflipped.coeffs()(1), corr_plane1_pair.plane_unflipped.coeffs()(2), corr_plane1_pair.plane_unflipped.coeffs()(3));
    ROS_DEBUG_NAMED("corridor planes", "final corridor plane 2 %f %f %f %f", corr_plane2_pair.plane_unflipped.coeffs()(0), corr_plane2_pair.plane_unflipped.coeffs()(1), corr_plane2_pair.plane_unflipped.coeffs()(2), corr_plane2_pair.plane_unflipped.coeffs()(3));

    if(plane_type == plane_class::X_VERT_PLANE) {
      auto found_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
      auto found_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);

      corr_data_association = associate_corridors(plane_type, corr_pose, (*found_plane1), (*found_plane2));

      if((x_corridors.empty() || corr_data_association.first == -1)) {
        std::cout << "found an X corridor with pre pose " << corr_pose << " between plane id " << corr_plane1_pair.plane_id << " and plane id " << corr_plane2_pair.plane_id << std::endl;

        corr_data_association.first = graph_slam->num_vertices_local();
        corr_node = graph_slam->add_corridor_node(corr_pose);
        // corr_node->setFixed(true);
        Corridors det_corridor;
        det_corridor.id = corr_data_association.first;
        det_corridor.plane1 = corr_plane1_pair.plane_unflipped;
        det_corridor.plane2 = corr_plane2_pair.plane_unflipped;
        det_corridor.plane1_id = corr_plane1_pair.plane_id;
        det_corridor.plane2_id = corr_plane2_pair.plane_id;
        det_corridor.keyframe_trans = corr_plane1_pair.keyframe_node->estimate().translation().head(3);
        det_corridor.node = corr_node;
        x_corridors.push_back(det_corridor);

        meas_plane1 = corridor_measurement(plane_type, corr_pose, corr_plane1_pair.plane_unflipped.coeffs());
        meas_plane2 = corridor_measurement(plane_type, corr_pose, corr_plane2_pair.plane_unflipped.coeffs());
        /* Add parallel constraints here */
        if(use_parallel_plane_constraint) {
          parallel_plane_constraint((*found_plane1).plane_node, (*found_plane2).plane_node);
        }

      } else {
        /* add the edge between detected planes and the corridor */
        corr_node = x_corridors[corr_data_association.second].node;
        std::cout << "Matched det corridor X with pre pose " << corr_pose << " to mapped corridor with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

        auto found_mapped_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[corr_data_association.second].plane1_id);
        auto found_mapped_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[corr_data_association.second].plane2_id);

        Eigen::Vector4d found_mapped_plane1_coeffs, found_mapped_plane2_coeffs;
        found_mapped_plane1_coeffs = (*found_mapped_plane1).plane_node->estimate().coeffs();
        found_mapped_plane2_coeffs = (*found_mapped_plane2).plane_node->estimate().coeffs();
        std::cout << "found_mapped_xplane1_coeffs: " << found_mapped_plane1_coeffs << std::endl;
        std::cout << "found_mapped_xplane2_coeffs: " << found_mapped_plane2_coeffs << std::endl;
        correct_plane_d(plane_class::X_VERT_PLANE, found_mapped_plane1_coeffs);
        correct_plane_d(plane_class::X_VERT_PLANE, found_mapped_plane2_coeffs);

        bool found_new_plane = false;
        if((*found_plane1).id == (*found_mapped_plane1).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane1).id == (*found_mapped_plane2).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane1).plane_node->estimate().coeffs().head(3)).dot(found_mapped_plane1_coeffs.head(3)) > 0) {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane1);
          } else {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_x_vert_planes.push_back(dupl_plane_pair);
        }

        if((*found_plane2).id == (*found_mapped_plane1).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane2).id == (*found_mapped_plane2).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane2).plane_node->estimate().coeffs().head(3)).dot(found_mapped_plane1_coeffs.head(3)) > 0) {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane1);
          } else {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_x_vert_planes.push_back(dupl_plane_pair);
        }

        std::cout << "x mapped plane1 id : " << (*found_mapped_plane1).id << std::endl;
        std::cout << "x mapped plane2 id : " << (*found_mapped_plane2).id << std::endl;
        std::cout << "x found plane1 id : " << (*found_plane1).id << std::endl;
        std::cout << "x found plane2 id : " << (*found_plane2).id << std::endl;

        if(use_parallel_plane_constraint && found_new_plane) {
          parallel_plane_constraint((*found_plane1).plane_node, (*found_plane2).plane_node);
        }
      }

      auto edge_plane1 = graph_slam->add_corridor_xplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
      graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);

      auto edge_plane2 = graph_slam->add_corridor_xplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
      graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      auto found_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane1_pair.plane_id);
      auto found_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == corr_plane2_pair.plane_id);
      corr_data_association = associate_corridors(plane_type, corr_pose, (*found_plane1), (*found_plane2));

      if((y_corridors.empty() || corr_data_association.first == -1)) {
        std::cout << "found an Y corridor with pre pose " << corr_pose << " between plane id " << corr_plane1_pair.plane_unflipped.coeffs() << " and plane id " << corr_plane2_pair.plane_unflipped.coeffs() << std::endl;

        corr_data_association.first = graph_slam->num_vertices_local();
        corr_node = graph_slam->add_corridor_node(corr_pose);
        // corr_node->setFixed(true);
        Corridors det_corridor;
        det_corridor.id = corr_data_association.first;
        det_corridor.plane1 = corr_plane1_pair.plane_unflipped;
        det_corridor.plane2 = corr_plane2_pair.plane_unflipped;
        det_corridor.plane1_id = corr_plane1_pair.plane_id;
        det_corridor.plane2_id = corr_plane2_pair.plane_id;
        det_corridor.keyframe_trans = corr_plane1_pair.keyframe_node->estimate().translation().head(3);
        det_corridor.node = corr_node;
        y_corridors.push_back(det_corridor);

        meas_plane1 = corridor_measurement(plane_type, corr_pose, corr_plane1_pair.plane_unflipped.coeffs());
        meas_plane2 = corridor_measurement(plane_type, corr_pose, corr_plane2_pair.plane_unflipped.coeffs());

        if(use_parallel_plane_constraint) {
          parallel_plane_constraint((*found_plane1).plane_node, (*found_plane2).plane_node);
        }

      } else {
        /* add the edge between detected planes and the corridor */
        corr_node = y_corridors[corr_data_association.second].node;
        std::cout << "Matched det corridor Y with pre pose " << corr_pose << " to mapped corridor with id " << corr_data_association.first << " and pose " << corr_node->estimate() << std::endl;

        auto found_mapped_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[corr_data_association.second].plane1_id);
        auto found_mapped_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[corr_data_association.second].plane2_id);

        Eigen::Vector4d found_mapped_plane1_coeffs, found_mapped_plane2_coeffs;
        found_mapped_plane1_coeffs = (*found_mapped_plane1).plane_node->estimate().coeffs();
        found_mapped_plane2_coeffs = (*found_mapped_plane2).plane_node->estimate().coeffs();
        correct_plane_d(plane_class::Y_VERT_PLANE, found_mapped_plane1_coeffs);
        correct_plane_d(plane_class::Y_VERT_PLANE, found_mapped_plane2_coeffs);

        bool found_new_plane = false;
        if((*found_plane1).id == (*found_mapped_plane1).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane1).id == (*found_mapped_plane2).id)
          meas_plane1 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane1).plane_node->estimate().coeffs().head(3)).dot(found_mapped_plane1_coeffs.head(3)) > 0) {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane1);
          } else {
            meas_plane1 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane1, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_y_vert_planes.push_back(dupl_plane_pair);
        }

        if((*found_plane2).id == (*found_mapped_plane1).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane1_coeffs);
        else if((*found_plane2).id == (*found_mapped_plane2).id)
          meas_plane2 = corridor_measurement(plane_type, corr_pose, found_mapped_plane2_coeffs);
        else {
          std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
          if(((*found_plane2).plane_node->estimate().coeffs().head(3)).dot(found_mapped_plane1_coeffs.head(3)) > 0) {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane1_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane1);
          } else {
            meas_plane2 = corridor_measurement(plane_type, corr_node->estimate(), found_mapped_plane2_coeffs);
            dupl_plane_pair = std::make_pair(*found_plane2, *found_mapped_plane2);
          }
          found_new_plane = true;
          dupl_y_vert_planes.push_back(dupl_plane_pair);
        }

        // std::cout << "y mapped plane1 id : " << (*found_mapped_plane1).id << std::endl;
        // std::cout << "y mapped plane2 id : " << (*found_mapped_plane2).id << std::endl;
        // std::cout << "y found plane1 id : " << (*found_plane1).id << std::endl;
        // std::cout << "y found plane2 id : " << (*found_plane2).id << std::endl;

        if(use_parallel_plane_constraint && found_new_plane) {
          parallel_plane_constraint((*found_plane1).plane_node, (*found_plane2).plane_node);
        }
      }

      auto edge_plane1 = graph_slam->add_corridor_yplane_edge(corr_node, (*found_plane1).plane_node, meas_plane1, information_corridor_plane);
      graph_slam->add_robust_kernel(edge_plane1, "Huber", 1.0);

      auto edge_plane2 = graph_slam->add_corridor_yplane_edge(corr_node, (*found_plane2).plane_node, meas_plane2, information_corridor_plane);
      graph_slam->add_robust_kernel(edge_plane2, "Huber", 1.0);
    }

    return;
  }

  double compute_corridor_pose(int plane_type, Eigen::Vector4d v1, Eigen::Vector4d v2) {
    double corridor_pose = 0;

    if(fabs(v1(3)) > fabs(v2(3))) {
      double size = v1(3) - v2(3);
      corridor_pose = ((size) / 2) + v2(3);
    } else {
      double size = v2(3) - v1(3);
      corridor_pose = ((size) / 2) + v1(3);
    }

    return corridor_pose;
  }

  double corridor_measurement(int plane_type, double corr, Eigen::Vector4d plane) {
    double meas = 0;

    if(fabs(corr) > fabs(plane(3))) {
      meas = corr - plane(3);
    } else {
      meas = plane(3) - corr;
    }

    return meas;
  }

  std::pair<int, int> associate_corridors(int plane_type, double corr_pose, VerticalPlanes plane1, VerticalPlanes plane2) {
    float min_dist = 100;
    float plane1_min_segment = 100, plane2_min_segment = 100;

    std::pair<int, int> data_association;
    data_association.first = -1;

    if(plane_type == plane_class::X_VERT_PLANE) {
      for(int i = 0; i < x_corridors.size(); ++i) {
        float dist = fabs((corr_pose) - (x_corridors[i].node->estimate()));

        auto found_mapped_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[i].plane1_id);
        auto found_mapped_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridors[i].plane2_id);

        if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
          plane1_min_segment = 0.0;
        } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
          plane1_min_segment = get_min_segment((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        } else
          plane1_min_segment = get_min_segment((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);

        if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
          plane2_min_segment = 0.0;
        } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
          plane2_min_segment = get_min_segment((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        } else
          plane2_min_segment = get_min_segment((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);

        if(dist < min_dist && (plane1_min_segment < 0.5 && plane2_min_segment < 0.5)) {
          min_dist = dist;
          data_association.first = x_corridors[i].id;
          data_association.second = i;
          ROS_DEBUG_NAMED("corridor planes", "dist x corr %f", dist);
        }
      }
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      for(int i = 0; i < y_corridors.size(); ++i) {
        float dist = fabs((corr_pose) - (y_corridors[i].node->estimate()));

        auto found_mapped_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[i].plane1_id);
        auto found_mapped_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridors[i].plane2_id);

        if(plane1.id == (*found_mapped_plane1).id || plane1.id == (*found_mapped_plane2).id) {
          plane1_min_segment = 0.0;
        } else if((plane1).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
          plane1_min_segment = get_min_segment((*found_mapped_plane1).cloud_seg_map, plane1.cloud_seg_map);
        } else
          plane1_min_segment = get_min_segment((*found_mapped_plane2).cloud_seg_map, plane1.cloud_seg_map);

        if(plane2.id == (*found_mapped_plane1).id || plane2.id == (*found_mapped_plane2).id) {
          plane2_min_segment = 0.0;
        } else if((plane2).plane_node->estimate().coeffs().head(3).dot((*found_mapped_plane1).plane_node->estimate().coeffs().head(3)) > 0) {
          plane2_min_segment = get_min_segment((*found_mapped_plane1).cloud_seg_map, plane2.cloud_seg_map);
        } else
          plane2_min_segment = get_min_segment((*found_mapped_plane2).cloud_seg_map, plane2.cloud_seg_map);

        if(dist < min_dist && (plane1_min_segment < 0.5 && plane2_min_segment < 0.5)) {
          min_dist = dist;
          data_association.first = y_corridors[i].id;
          data_association.second = i;
          ROS_DEBUG_NAMED("corridor planes", "dist y corr %f", dist);
        }
      }
    }

    // ROS_DEBUG_NAMED("corridor planes", "min dist %f", min_dist);
    if(min_dist > corridor_dist_threshold) data_association.first = -1;

    return data_association;
  }

  /**
   * @brief this method creates the room vertex and adds edges between the vertex and detected planes
   */
  void factor_rooms(std::vector<plane_data_list> x_room_pair_vec, std::vector<plane_data_list> y_room_pair_vec) {
    g2o::VertexRoomXYLB* room_node;
    std::pair<int, int> room_data_association;
    Eigen::Matrix<double, 1, 1> information_room_plane(room_information);
    auto found_x_plane1 = x_vert_planes.begin();
    auto found_x_plane2 = x_vert_planes.begin();
    auto found_y_plane1 = y_vert_planes.begin();
    auto found_y_plane2 = y_vert_planes.begin();
    auto found_mapped_x_plane1 = x_vert_planes.begin();
    auto found_mapped_x_plane2 = x_vert_planes.begin();
    auto found_mapped_y_plane1 = y_vert_planes.begin();
    auto found_mapped_y_plane2 = y_vert_planes.begin();
    double x_plane1_meas, x_plane2_meas;
    double y_plane1_meas, y_plane2_meas;

    ROS_DEBUG_NAMED("room planes", "final room plane 1 %f %f %f %f", x_room_pair_vec[0].plane_unflipped.coeffs()(0), x_room_pair_vec[0].plane_unflipped.coeffs()(1), x_room_pair_vec[0].plane_unflipped.coeffs()(2), x_room_pair_vec[0].plane_unflipped.coeffs()(3));
    ROS_DEBUG_NAMED("room planes", "final room plane 2 %f %f %f %f", x_room_pair_vec[1].plane_unflipped.coeffs()(0), x_room_pair_vec[1].plane_unflipped.coeffs()(1), x_room_pair_vec[1].plane_unflipped.coeffs()(2), x_room_pair_vec[1].plane_unflipped.coeffs()(3));
    ROS_DEBUG_NAMED("room planes", "final room plane 3 %f %f %f %f", y_room_pair_vec[0].plane_unflipped.coeffs()(0), y_room_pair_vec[0].plane_unflipped.coeffs()(1), y_room_pair_vec[0].plane_unflipped.coeffs()(2), y_room_pair_vec[0].plane_unflipped.coeffs()(3));
    ROS_DEBUG_NAMED("room planes", "final room plane 4 %f %f %f %f", y_room_pair_vec[1].plane_unflipped.coeffs()(0), y_room_pair_vec[1].plane_unflipped.coeffs()(1), y_room_pair_vec[1].plane_unflipped.coeffs()(2), y_room_pair_vec[1].plane_unflipped.coeffs()(3));

    found_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_room_pair_vec[0].plane_id);
    found_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == x_room_pair_vec[1].plane_id);
    found_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_room_pair_vec[0].plane_id);
    found_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == y_room_pair_vec[1].plane_id);

    Eigen::Vector2d room_pose = compute_room_pose(x_room_pair_vec, y_room_pair_vec);
    room_data_association = associate_rooms(room_pose);
    if((rooms_vec.empty() || room_data_association.first == -1)) {
      std::cout << "found room with pose " << room_pose << std::endl;
      room_data_association.first = graph_slam->num_vertices_local();
      room_node = graph_slam->add_room_node(room_pose);
      // room_node->setFixed(true);
      Rooms det_room;
      det_room.id = room_data_association.first;
      det_room.plane_x1 = x_room_pair_vec[0].plane_unflipped;
      det_room.plane_x2 = x_room_pair_vec[1].plane_unflipped;
      det_room.plane_y1 = y_room_pair_vec[0].plane_unflipped;
      det_room.plane_y2 = y_room_pair_vec[1].plane_unflipped;
      det_room.plane_x1_id = x_room_pair_vec[0].plane_id;
      det_room.plane_x2_id = x_room_pair_vec[1].plane_id;
      det_room.plane_y1_id = y_room_pair_vec[0].plane_id;
      det_room.plane_y2_id = y_room_pair_vec[1].plane_id;
      det_room.node = room_node;
      rooms_vec.push_back(det_room);

      x_plane1_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, x_room_pair_vec[0].plane_unflipped.coeffs());
      x_plane2_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, x_room_pair_vec[1].plane_unflipped.coeffs());

      y_plane1_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, y_room_pair_vec[0].plane_unflipped.coeffs());
      y_plane2_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, y_room_pair_vec[1].plane_unflipped.coeffs());

      /* Add parallel and perpendicular constraints here */
      if(use_parallel_plane_constraint) {
        parallel_plane_constraint((*found_x_plane1).plane_node, (*found_x_plane2).plane_node);
        parallel_plane_constraint((*found_y_plane1).plane_node, (*found_y_plane2).plane_node);
      }
      if(use_perpendicular_plane_constraint) {
        perpendicular_plane_constraint((*found_x_plane1).plane_node, (*found_y_plane1).plane_node);
        perpendicular_plane_constraint((*found_x_plane1).plane_node, (*found_y_plane2).plane_node);
        perpendicular_plane_constraint((*found_x_plane2).plane_node, (*found_y_plane1).plane_node);
        perpendicular_plane_constraint((*found_x_plane2).plane_node, (*found_y_plane2).plane_node);
      }

    } else {
      /* add the edge between detected planes and the corridor */
      room_node = rooms_vec[room_data_association.second].node;
      std::cout << "Matched det room with pose " << room_pose << " to mapped room with id " << room_data_association.first << " and pose " << room_node->estimate() << std::endl;

      found_mapped_x_plane1 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_x1_id);
      found_mapped_x_plane2 = std::find_if(x_vert_planes.begin(), x_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_x2_id);
      Eigen::Vector4d found_mapped_x_plane1_coeffs, found_mapped_x_plane2_coeffs;
      found_mapped_x_plane1_coeffs = (*found_mapped_x_plane1).plane_node->estimate().coeffs();
      found_mapped_x_plane2_coeffs = (*found_mapped_x_plane2).plane_node->estimate().coeffs();
      correct_plane_d(plane_class::X_VERT_PLANE, found_mapped_x_plane1_coeffs);
      correct_plane_d(plane_class::X_VERT_PLANE, found_mapped_x_plane2_coeffs);

      bool found_new_x_plane = false;
      if((*found_x_plane1).id == (*found_mapped_x_plane1).id)
        x_plane1_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane1_coeffs);
      else if((*found_x_plane1).id == (*found_mapped_x_plane2).id)
        x_plane1_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane2_coeffs);
      else {
        std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
        if((*found_x_plane1).plane_node->estimate().coeffs().head(3).dot(found_mapped_x_plane1_coeffs.head(3)) > 0) {
          x_plane1_meas = room_measurement(plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane1_coeffs);
          dupl_plane_pair = std::make_pair(*found_x_plane1, *found_mapped_x_plane1);
        } else {
          x_plane1_meas = room_measurement(plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane2_coeffs);
          dupl_plane_pair = std::make_pair(*found_x_plane1, *found_mapped_x_plane2);
        }
        found_new_x_plane = true;
        dupl_x_vert_planes.push_back(dupl_plane_pair);
      }

      if((*found_x_plane2).id == (*found_mapped_x_plane1).id)
        x_plane2_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane1_coeffs);
      else if((*found_x_plane2).id == (*found_mapped_x_plane2).id)
        x_plane2_meas = room_measurement(plane_class::X_VERT_PLANE, room_pose, found_mapped_x_plane2_coeffs);
      else {
        std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
        if((*found_x_plane2).plane_node->estimate().coeffs().head(3).dot(found_mapped_x_plane1_coeffs.head(3)) > 0) {
          x_plane2_meas = room_measurement(plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane1_coeffs);
          dupl_plane_pair = std::make_pair(*found_x_plane2, *found_mapped_x_plane1);
        } else {
          x_plane2_meas = room_measurement(plane_class::X_VERT_PLANE, room_node->estimate(), found_mapped_x_plane2_coeffs);
          dupl_plane_pair = std::make_pair(*found_x_plane2, *found_mapped_x_plane2);
        }
        found_new_x_plane = true;
        dupl_x_vert_planes.push_back(dupl_plane_pair);
      }

      if(use_parallel_plane_constraint && found_new_x_plane) {
        parallel_plane_constraint((*found_x_plane1).plane_node, (*found_x_plane2).plane_node);
      }

      found_mapped_y_plane1 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_y1_id);
      found_mapped_y_plane2 = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == rooms_vec[room_data_association.second].plane_y2_id);
      Eigen::Vector4d found_mapped_y_plane1_coeffs, found_mapped_y_plane2_coeffs;
      found_mapped_y_plane1_coeffs = (*found_mapped_y_plane1).plane_node->estimate().coeffs();
      found_mapped_y_plane2_coeffs = (*found_mapped_y_plane2).plane_node->estimate().coeffs();
      correct_plane_d(plane_class::Y_VERT_PLANE, found_mapped_y_plane1_coeffs);
      correct_plane_d(plane_class::Y_VERT_PLANE, found_mapped_y_plane2_coeffs);

      bool found_new_y_plane = false;
      if((*found_y_plane1).id == (*found_mapped_y_plane1).id)
        y_plane1_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane1_coeffs);
      else if((*found_y_plane1).id == (*found_mapped_y_plane2).id)
        y_plane1_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane2_coeffs);
      else {
        std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
        if((*found_y_plane1).plane_node->estimate().coeffs().head(3).dot(found_mapped_y_plane1_coeffs.head(3)) > 0) {
          y_plane1_meas = room_measurement(plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane1_coeffs);
          dupl_plane_pair = std::make_pair(*found_y_plane1, *found_mapped_y_plane1);
        } else {
          y_plane1_meas = room_measurement(plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane2_coeffs);
          dupl_plane_pair = std::make_pair(*found_y_plane1, *found_mapped_y_plane2);
        }
        found_new_y_plane = true;
        dupl_y_vert_planes.push_back(dupl_plane_pair);
      }

      if((*found_y_plane2).id == (*found_mapped_y_plane1).id)
        y_plane2_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane1_coeffs);
      else if((*found_y_plane2).id == (*found_mapped_y_plane2).id)
        y_plane2_meas = room_measurement(plane_class::Y_VERT_PLANE, room_pose, found_mapped_y_plane2_coeffs);
      else {
        std::pair<VerticalPlanes, VerticalPlanes> dupl_plane_pair;
        if((*found_y_plane2).plane_node->estimate().coeffs().head(3).dot(found_mapped_y_plane1_coeffs.head(3)) > 0) {
          y_plane2_meas = room_measurement(plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane1_coeffs);
          dupl_plane_pair = std::make_pair(*found_y_plane2, *found_mapped_y_plane1);
        } else {
          y_plane2_meas = room_measurement(plane_class::Y_VERT_PLANE, room_node->estimate(), found_mapped_y_plane2_coeffs);
          dupl_plane_pair = std::make_pair(*found_y_plane2, *found_mapped_y_plane2);
        }
        found_new_y_plane = true;
        dupl_y_vert_planes.push_back(dupl_plane_pair);
      }

      if(use_parallel_plane_constraint && found_new_y_plane) {
        parallel_plane_constraint((*found_y_plane1).plane_node, (*found_y_plane2).plane_node);
      }
    }

    // std::cout << "found xplane1 id : " << (*found_x_plane1).id << std::endl;
    // std::cout << "found xplane2 id : " << (*found_x_plane2).id << std::endl;
    // std::cout << "mapped xplane1 id : " << (*found_mapped_x_plane1).id << std::endl;
    // std::cout << "mapped xplane2 id : " << (*found_mapped_x_plane2).id << std::endl;
    // std::cout << "found yplane1 id : " << (*found_y_plane1).id << std::endl;
    // std::cout << "found yplane2 id : " << (*found_y_plane2).id << std::endl;
    // std::cout << "mapped yplane1 id : " << (*found_mapped_y_plane1).id << std::endl;
    // std::cout << "mapped yplane2 id : " << (*found_mapped_y_plane2).id << std::endl;

    auto edge_x_plane1 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane1).plane_node, x_plane1_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_x_plane1, "Huber", 1.0);

    auto edge_x_plane2 = graph_slam->add_room_xplane_edge(room_node, (*found_x_plane2).plane_node, x_plane2_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_x_plane2, "Huber", 1.0);

    auto edge_y_plane1 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane1).plane_node, y_plane1_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_y_plane1, "Huber", 1.0);

    auto edge_y_plane2 = graph_slam->add_room_yplane_edge(room_node, (*found_y_plane2).plane_node, y_plane2_meas, information_room_plane);
    graph_slam->add_robust_kernel(edge_y_plane2, "Huber", 1.0);
  }

  Eigen::Vector2d compute_room_pose(std::vector<plane_data_list> x_room_pair_vec, std::vector<plane_data_list> y_room_pair_vec) {
    Eigen::Vector2d room_pose(0, 0);
    Eigen::Vector4d x_plane1 = x_room_pair_vec[0].plane_unflipped.coeffs(), x_plane2 = x_room_pair_vec[1].plane_unflipped.coeffs();
    Eigen::Vector4d y_plane1 = y_room_pair_vec[0].plane_unflipped.coeffs(), y_plane2 = y_room_pair_vec[1].plane_unflipped.coeffs();

    if(fabs(x_plane1(3)) > fabs(x_plane2(3))) {
      double size = x_plane1(3) - x_plane2(3);
      room_pose(0) = (((size) / 2) + x_plane2(3));
      // room_pose(2) = size;
    } else {
      double size = x_plane2(3) - x_plane1(3);
      room_pose(0) = (((size) / 2) + x_plane1(3));
      // room_pose(2) = size;
    }

    if(fabs(y_plane1(3)) > fabs(y_plane2(3))) {
      double size = y_plane1(3) - y_plane2(3);
      room_pose(1) = (((size) / 2) + y_plane2(3));
      // room_pose(3) = size;
    } else {
      double size = y_plane2(3) - y_plane1(3);
      room_pose(1) = (((size) / 2) + y_plane1(3));
      // room_pose(3) = size;
    }

    return room_pose;
  }

  Eigen::Vector2d compute_room_pose_local(g2o::VertexSE3* keyframe_node, Eigen::Vector2d room_pose) {
    Eigen::Isometry3d room_pose_map;
    room_pose_map.matrix().block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    room_pose_map.matrix().block<2, 1>(0, 3) = room_pose;

    Eigen::Isometry3d room_pose_local;
    room_pose_local = room_pose_map * keyframe_node->estimate().inverse();

    return room_pose_local.matrix().block<2, 1>(0, 3);
  }

  double room_measurement(int plane_type, Eigen::Vector2d room, Eigen::Vector4d plane) {
    double meas;

    if(plane_type == plane_class::X_VERT_PLANE) {
      if(fabs(room(0)) > fabs(plane(3))) {
        meas = room(0) - plane(3);
      } else {
        meas = plane(3) - room(0);
      }
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      if(fabs(room(1)) > fabs(plane(3))) {
        meas = room(1) - plane(3);
      } else {
        meas = plane(3) - room(1);
      }
    }

    return meas;
  }

  std::pair<int, int> associate_rooms(Eigen::Vector2d room_pose) {
    float min_dist = 100;
    std::pair<int, int> data_association;
    data_association.first = -1;

    for(int i = 0; i < rooms_vec.size(); ++i) {
      float diff_x = room_pose(0) - rooms_vec[i].node->estimate()(0);
      float diff_y = room_pose(1) - rooms_vec[i].node->estimate()(1);
      float dist = sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
      ROS_DEBUG_NAMED("room planes", "dist room %f", dist);

      if(dist < min_dist) {
        min_dist = dist;
        data_association.first = rooms_vec[i].id;
        data_association.second = i;
      }
    }

    ROS_DEBUG_NAMED("room planes", "min dist room %f", min_dist);
    if(min_dist > room_dist_threshold) data_association.first = -1;

    return data_association;
  }

  /**
   * @brief this method add parallel constraint between the planes of rooms or corridors
   */
  void parallel_plane_constraint(g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_parallel_edge(plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  /**
   * @brief this method adds perpendicular constraint between the planes of rooms or corridors
   */
  void perpendicular_plane_constraint(g2o::VertexPlane* plane1_node, g2o::VertexPlane* plane2_node) {
    Eigen::Matrix<double, 1, 1> information(0.1);
    Eigen::Vector3d meas(0, 0, 0);

    auto edge = graph_slam->add_plane_perpendicular_edge(plane1_node, plane2_node, meas, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }

  float plane_length(pcl::PointCloud<PointNormal>::Ptr cloud_seg, pcl::PointXY& p1, pcl::PointXY& p2, g2o::VertexSE3* keyframe_node) {
    PointNormal pmin, pmax;
    pcl::getMaxSegment(*cloud_seg, pmin, pmax);
    p1.x = pmin.x;
    p1.y = pmin.y;
    p2.x = pmax.x;
    p2.y = pmax.y;
    float length = pcl::euclideanDistance(p1, p2);

    pcl::PointXY p1_map, p2_map;
    p1_map = convert_point_to_map(p1, keyframe_node->estimate().matrix());
    p2_map = convert_point_to_map(p2, keyframe_node->estimate().matrix());
    p1 = p1_map;
    p2 = p2_map;

    return length;
  }

  float get_min_segment(const pcl::PointCloud<PointNormal>::Ptr& cloud_1, const pcl::PointCloud<PointNormal>::Ptr& cloud_2) {
    float min_dist = std::numeric_limits<float>::max();
    const auto token = std::numeric_limits<std::size_t>::max();
    std::size_t i_min = token, i_max = token;

    for(std::size_t i = 0; i < cloud_1->points.size(); ++i) {
      for(std::size_t j = 0; j < cloud_2->points.size(); ++j) {
        // Compute the distance
        float dist = (cloud_1->points[i].getVector4fMap() - cloud_2->points[j].getVector4fMap()).squaredNorm();
        if(dist >= min_dist) continue;

        min_dist = dist;
        i_min = i;
        i_max = j;
      }
    }

    // if (i_min == token || i_max == token)
    //  return (min_dist = std::numeric_limits<double>::min ());

    // pmin = cloud.points[i_min];
    // pmax = cloud.points[i_max];
    return (std::sqrt(min_dist));
  }

  float width_between_planes(Eigen::Vector4d v1, Eigen::Vector4d v2) {
    float size = 0;
    if(fabs(v1(3)) > fabs(v2(3)))
      size = fabs(v1(3) - v2(3));
    else if(fabs(v2(3)) > fabs(v1(3)))
      size = fabs(v2(3) - v1(3));

    return size;
  }

  float point_difference(int plane_type, pcl::PointXY p1, pcl::PointXY p2) {
    float point_diff = 0;

    if(plane_type == plane_class::X_VERT_PLANE) {
      p1.x = 0;
      p2.x = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }
    if(plane_type == plane_class::Y_VERT_PLANE) {
      p1.y = 0;
      p2.y = 0;
      point_diff = pcl::euclideanDistance(p1, p2);
    }

    return point_diff;
  }

  pcl::PointXY convert_point_to_map(pcl::PointXY point_local, Eigen::Matrix4d keyframe_pose) {
    pcl::PointXY point_map;

    Eigen::Vector4d point_map_eigen, point_local_eigen;
    point_local_eigen = point_map_eigen.setZero();
    point_local_eigen(3) = point_map_eigen(3) = 1;
    point_local_eigen(0) = point_local.x;
    point_local_eigen(1) = point_local.y;
    point_map_eigen = keyframe_pose * point_local_eigen;

    point_map.x = point_map_eigen(0);
    point_map.y = point_map_eigen(1);
    return point_map;
  }

  void correct_plane_d(int plane_type, Eigen::Vector4d& plane) {
    if(plane_type == plane_class::X_VERT_PLANE) {
      plane(3) = -1 * plane(3);
      double p_norm = plane(0) / fabs(plane(0));
      plane(3) = p_norm * plane(3);
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      plane(3) = -1 * plane(3);
      double p_norm = plane(1) / fabs(plane(1));
      plane(3) = p_norm * plane(3);
    }
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    std::vector<plane_data_list> x_det_corridor_candidates, y_det_corridor_candidates;
    std::vector<plane_data_list> x_det_room_candidates, y_det_room_candidates;
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

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));

      // Segment the planes and rooms here
      std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec = plane_analyzer->extract_segmented_planes(keyframe->cloud);

      // From all the current segmented planes at a keyframe detect rooms/corridors
      for(const auto& cloud_seg_body : extracted_cloud_vec) {
        if(cloud_seg_body->points.size() < min_plane_points) continue;

        keyframe->cloud_seg_body = cloud_seg_body;
        g2o::Plane3D det_plane_body_frame = Eigen::Vector4d(cloud_seg_body->back().normal_x, cloud_seg_body->back().normal_y, cloud_seg_body->back().normal_z, cloud_seg_body->back().curvature);
        bool found_corridor_candidates = false;
        bool found_room_candidates = false;
        plane_data_list plane_id_pair;

        int plane_type = map_detected_planes(keyframe, det_plane_body_frame, found_corridor_candidates, found_room_candidates, plane_id_pair);

        switch(plane_type) {
          case plane_class::X_VERT_PLANE: {
            if(found_corridor_candidates) {
              x_det_corridor_candidates.push_back(plane_id_pair);
            }
            if(found_room_candidates) {
              x_det_room_candidates.push_back(plane_id_pair);
            }
            break;
          }
          case plane_class::Y_VERT_PLANE: {
            if(found_corridor_candidates) {
              y_det_corridor_candidates.push_back(plane_id_pair);
            }
            if(found_room_candidates) {
              y_det_room_candidates.push_back(plane_id_pair);
            }
            break;
          }
          case plane_class::HORT_PLANE: {
            break;
          }
          default: {
            break;
          }
        }
      }
    }

    if(use_corridor_constraint) {
      // std::cout << "x_det_corridor_candidates: " << x_det_corridor_candidates.size() << std::endl;sg
      // std::cout << "y_det_corridor_candidates: " << y_det_corridor_candidates.size() << std::endl;
      // lookup_corridors(x_det_corridor_candidates, y_det_corridor_candidates);
    }

    if(use_room_constraint) {
      // std::cout << "x_det_room_candidates: " << x_det_room_candidates.size() << std::endl;
      // std::cout << "y_det_room_candidates: " << y_det_room_candidates.size() << std::endl;
      // lookup_rooms(x_det_room_candidates, y_det_room_candidates);
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
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
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

    auto markers = create_marker_array(ros::Time::now());
    markers_pub.publish(markers);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::WallTimerEvent& event) {
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

    publish_mapped_planes(x_vert_planes, y_vert_planes);

    // flush the room poses from room detector and no need to return if no rooms found
    flush_room_data_queue();

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
      // correct_plane_direction(PlaneUtils::plane_class::X_VERT_PLANE, mapped_plane_coeffs);
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
      // correct_plane_direction(PlaneUtils::plane_class::Y_VERT_PLANE, mapped_plane_coeffs);
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
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) {
    visualization_msgs::MarkerArray markers;
    // markers.markers.resize(11);

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

    // node markers
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
      traj_marker.points[i].z = pos.z();

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

    // edge markers
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = map_frame_id;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = markers.markers.size();
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(local_graph->edges().size() * 6);
    edge_marker.colors.resize(local_graph->edges().size() * 6);

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
        } else if(fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(0)) && fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(2))) {
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
      sphere_marker.pose.position.z = pos.z();
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
      color.r = 1 - p;
      color.g = p;
      color.b = p;
      color.a = 0.5;
      for(size_t j = 0; j < x_plane_snapshot[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = x_plane_snapshot[i].cloud_seg_map->points[j].x;
        point.y = x_plane_snapshot[i].cloud_seg_map->points[j].y;
        point.z = x_plane_snapshot[i].cloud_seg_map->points[j].z + 5.0;
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
      color.r = 0;
      color.g = 1.0 - p;
      color.b = p;
      color.a = 0.5;
      for(size_t j = 0; j < y_plane_snapshot[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = y_plane_snapshot[i].cloud_seg_map->points[j].x;
        point.y = y_plane_snapshot[i].cloud_seg_map->points[j].y;
        point.z = y_plane_snapshot[i].cloud_seg_map->points[j].z + 5.0;
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
        point.z = hort_plane_snapshot[i].cloud_seg_map->points[j].z + 5.0;
        hort_plane_marker.points.push_back(point);
      }
      hort_plane_marker.color.r = 1;
      hort_plane_marker.color.g = 0.65;
      hort_plane_marker.color.a = 0.5;
    }
    markers.markers.push_back(hort_plane_marker);

    float corridor_node_h = 10.5;
    float corridor_text_h = 10;
    float corridor_edge_h = 9.5;
    float corridor_point_h = 5.0;
    // x corridor markers
    visualization_msgs::Marker corridor_marker;
    corridor_marker.pose.orientation.w = 1.0;
    corridor_marker.scale.x = 0.5;
    corridor_marker.scale.y = 0.5;
    corridor_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    corridor_marker.header.frame_id = map_frame_id;
    corridor_marker.header.stamp = stamp;
    corridor_marker.ns = "corridors";
    corridor_marker.id = markers.markers.size();
    corridor_marker.type = visualization_msgs::Marker::CUBE_LIST;
    corridor_marker.color.r = 0;
    corridor_marker.color.g = 1;
    corridor_marker.color.a = 1;

    for(int i = 0; i < x_corridor_snapshot.size(); ++i) {
      auto found_plane1 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridor_snapshot[i].plane1_id);
      auto found_plane2 = std::find_if(x_plane_snapshot.begin(), x_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == x_corridor_snapshot[i].plane2_id);

      // fill in the line marker
      visualization_msgs::Marker corr_x_line_marker;
      corr_x_line_marker.scale.x = 0.05;
      corr_x_line_marker.pose.orientation.w = 1.0;
      corr_x_line_marker.ns = "corridor_x_lines";
      corr_x_line_marker.header.frame_id = map_frame_id;
      corr_x_line_marker.header.stamp = stamp;
      corr_x_line_marker.id = markers.markers.size() + 1;
      corr_x_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      corr_x_line_marker.color.r = color_r;
      corr_x_line_marker.color.g = color_g;
      corr_x_line_marker.color.b = color_b;
      corr_x_line_marker.color.a = 0.4;
      geometry_msgs::Point p1, p2, p3;

      pcl::CentroidPoint<PointNormal> centroid_p1;
      for(int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
        centroid_p1.add((*found_plane1).cloud_seg_map->points[p]);
      }

      PointNormal cp1_pt;
      centroid_p1.get(cp1_pt);
      p1.x = x_corridor_snapshot[i].node->estimate();
      p1.y = cp1_pt.y;
      p1.z = corridor_edge_h;
      p2.x = cp1_pt.x;
      p2.y = cp1_pt.y;
      p2.z = 5.0;
      corr_x_line_marker.points.push_back(p1);
      corr_x_line_marker.points.push_back(p2);

      pcl::CentroidPoint<PointNormal> centroid_p2;
      for(int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
        centroid_p2.add((*found_plane2).cloud_seg_map->points[p]);
      }
      PointNormal cp2_pt;
      centroid_p2.get(cp2_pt);
      p3.x = cp2_pt.x;
      p3.y = cp2_pt.y;
      p3.z = 5.0;
      corr_x_line_marker.points.push_back(p1);
      corr_x_line_marker.points.push_back(p3);
      markers.markers.push_back(corr_x_line_marker);

      // corridor cube
      geometry_msgs::Point point;
      point.x = x_corridor_snapshot[i].node->estimate();
      point.y = cp1_pt.y;
      point.z = corridor_node_h;
      corridor_marker.points.push_back(point);

      // fill in the text marker
      visualization_msgs::Marker corr_x_text_marker;
      corr_x_text_marker.scale.z = 0.5;
      corr_x_text_marker.ns = "corridor_x_text";
      corr_x_text_marker.header.frame_id = map_frame_id;
      corr_x_text_marker.header.stamp = stamp;
      corr_x_text_marker.id = markers.markers.size() + 1;
      corr_x_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      corr_x_text_marker.pose.position.x = x_corridor_snapshot[i].node->estimate();
      corr_x_text_marker.pose.position.y = cp1_pt.y;
      corr_x_text_marker.pose.position.z = corridor_text_h;
      corr_x_text_marker.color.r = color_r;
      corr_x_text_marker.color.g = color_g;
      corr_x_text_marker.color.b = color_b;
      corr_x_text_marker.color.a = 1;
      corr_x_text_marker.pose.orientation.w = 1.0;
      corr_x_text_marker.text = "Corridor X" + std::to_string(i + 1);
      markers.markers.push_back(corr_x_text_marker);
    }

    for(int i = 0; i < y_corridors.size(); ++i) {
      auto found_plane1 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridor_snapshot[i].plane1_id);
      auto found_plane2 = std::find_if(y_plane_snapshot.begin(), y_plane_snapshot.end(), boost::bind(&VerticalPlanes::id, _1) == y_corridor_snapshot[i].plane2_id);

      // fill in the line marker
      visualization_msgs::Marker corr_y_line_marker;
      corr_y_line_marker.scale.x = 0.05;
      corr_y_line_marker.pose.orientation.w = 1.0;
      corr_y_line_marker.ns = "corridor_y_lines";
      corr_y_line_marker.header.frame_id = map_frame_id;
      corr_y_line_marker.header.stamp = stamp;
      corr_y_line_marker.id = markers.markers.size() + 1;
      corr_y_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      corr_y_line_marker.color.r = color_r;
      corr_y_line_marker.color.g = color_g;
      corr_y_line_marker.color.b = color_b;
      corr_y_line_marker.color.a = 0.4;
      geometry_msgs::Point p1, p2, p3;

      pcl::CentroidPoint<PointNormal> centroid_p1;
      for(int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
        centroid_p1.add((*found_plane1).cloud_seg_map->points[p]);
      }
      PointNormal cp1_pt;
      centroid_p1.get(cp1_pt);
      p1.x = cp1_pt.x;
      p1.y = y_corridor_snapshot[i].node->estimate();
      p1.z = corridor_edge_h;
      p2.x = cp1_pt.x;
      p2.y = cp1_pt.y;
      p2.z = 5.0;
      corr_y_line_marker.points.push_back(p1);
      corr_y_line_marker.points.push_back(p2);

      pcl::CentroidPoint<PointNormal> centroid_p2;
      for(int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
        centroid_p2.add((*found_plane2).cloud_seg_map->points[p]);
      }
      PointNormal cp2_pt;
      centroid_p2.get(cp2_pt);
      p3.x = cp2_pt.x;
      p3.y = cp2_pt.y;
      p3.z = 5.0;
      corr_y_line_marker.points.push_back(p1);
      corr_y_line_marker.points.push_back(p3);
      markers.markers.push_back(corr_y_line_marker);

      // corridor cube
      geometry_msgs::Point point;
      point.x = cp1_pt.x;
      point.y = y_corridor_snapshot[i].node->estimate();
      point.z = corridor_node_h;
      corridor_marker.points.push_back(point);

      // fill in the text marker
      visualization_msgs::Marker corr_y_text_marker;
      corr_y_text_marker.scale.z = 0.5;
      corr_y_text_marker.ns = "corridor_y_text";
      corr_y_text_marker.header.frame_id = map_frame_id;
      corr_y_text_marker.header.stamp = stamp;
      corr_y_text_marker.id = markers.markers.size() + 1;
      corr_y_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      corr_y_text_marker.pose.position.x = cp1_pt.x;
      corr_y_text_marker.pose.position.y = y_corridor_snapshot[i].node->estimate();
      corr_y_text_marker.pose.position.z = corridor_text_h;
      corr_y_text_marker.color.r = color_r;
      corr_y_text_marker.color.g = color_g;
      corr_y_text_marker.color.b = color_b;
      corr_y_text_marker.color.a = 1;
      corr_y_text_marker.pose.orientation.w = 1.0;
      corr_y_text_marker.text = "Corridor Y" + std::to_string(i + 1);
      markers.markers.push_back(corr_y_text_marker);
    }
    markers.markers.push_back(corridor_marker);

    // room markers
    float room_node_h = 10.5;
    float room_text_h = 10;
    float room_edge_h = 9.5;
    float room_point_h = 5.0;
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

    for(int i = 0; i < room_snapshot.size(); ++i) {
      geometry_msgs::Point point;
      point.x = room_snapshot[i].node->estimate()(0);
      point.y = room_snapshot[i].node->estimate()(1);
      point.z = room_node_h;
      room_marker.points.push_back(point);

      // fill in the text marker
      visualization_msgs::Marker room_text_marker;
      room_text_marker.scale.z = 0.5;
      room_text_marker.ns = "rooms_text";
      room_text_marker.header.frame_id = map_frame_id;
      room_text_marker.header.stamp = stamp;
      room_text_marker.id = markers.markers.size() + 1;
      room_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      room_text_marker.pose.position.x = room_snapshot[i].node->estimate()(0);
      room_text_marker.pose.position.y = room_snapshot[i].node->estimate()(1);
      room_text_marker.pose.position.z = room_text_h;
      room_text_marker.color.r = color_r;
      room_text_marker.color.g = color_g;
      room_text_marker.color.b = color_b;
      room_text_marker.color.a = 1;
      room_text_marker.pose.orientation.w = 1.0;
      room_text_marker.text = "Room" + std::to_string(i + 1);
      markers.markers.push_back(room_text_marker);

      // fill in the line marker
      visualization_msgs::Marker room_line_marker;
      room_line_marker.scale.x = 0.05;
      room_line_marker.pose.orientation.w = 1.0;
      room_line_marker.ns = "rooms_lines";
      room_line_marker.header.frame_id = map_frame_id;
      room_line_marker.header.stamp = stamp;
      room_line_marker.id = markers.markers.size() + 1;
      room_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      room_line_marker.color.r = color_r;
      room_line_marker.color.g = color_g;
      room_line_marker.color.b = color_b;
      room_line_marker.color.a = 0.4;
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
        p_tmp.z = corridor_point_h;

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
        p_tmp.z = corridor_point_h;

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
        p_tmp.z = corridor_point_h;

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
        p_tmp.z = corridor_point_h;

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

    // //final line markers for printing different layers for abstraction
    // visualization_msgs::Marker robot_layer_marker;
    // robot_layer_marker.scale.z = 1.5;
    // robot_layer_marker.ns = "layer_marker";
    // robot_layer_marker.header.frame_id = map_frame_id;
    // robot_layer_marker.header.stamp = stamp;
    // robot_layer_marker.id = markers.markers.size();
    // robot_layer_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // robot_layer_marker.pose.position.x = 0.0;
    // robot_layer_marker.pose.position.y = 30.0;
    // robot_layer_marker.pose.position.z = 0.0;
    // robot_layer_marker.color.a = 1;
    // robot_layer_marker.pose.orientation.w = 1.0;
    // robot_layer_marker.color.r = color_r;
    // robot_layer_marker.color.g = color_g;
    // robot_layer_marker.color.b = color_b;
    // robot_layer_marker.text = "Robot Tracking Layer";
    // markers.markers.push_back(robot_layer_marker);

    // if(!y_plane_snapshot.empty() || !x_plane_snapshot.empty()) {
    //   visualization_msgs::Marker semantic_layer_marker;
    //   semantic_layer_marker.scale.z = 1.5;
    //   semantic_layer_marker.ns = "layer_marker";
    //   semantic_layer_marker.header.frame_id = map_frame_id;
    //   semantic_layer_marker.header.stamp = stamp;
    //   semantic_layer_marker.id = markers.markers.size();
    //   semantic_layer_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //   semantic_layer_marker.pose.position.x = 0.0;
    //   semantic_layer_marker.pose.position.y = 30.0;
    //   semantic_layer_marker.pose.position.z = 5.0;
    //   semantic_layer_marker.color.r = color_r;
    //   semantic_layer_marker.color.g = color_g;
    //   semantic_layer_marker.color.b = color_b;
    //   semantic_layer_marker.color.a = 1;
    //   semantic_layer_marker.pose.orientation.w = 1.0;
    //   semantic_layer_marker.text = "Metric-Semantic Layer";
    //   markers.markers.push_back(semantic_layer_marker);
    // }

    // if(!x_corridor_snapshot.empty() || !y_corridor_snapshot.empty() || !room_snapshot.empty()) {
    //   visualization_msgs::Marker topological_layer_marker;
    //   topological_layer_marker.scale.z = 1.5;
    //   topological_layer_marker.ns = "layer_marker";
    //   topological_layer_marker.header.frame_id = map_frame_id;
    //   topological_layer_marker.header.stamp = stamp;
    //   topological_layer_marker.id = markers.markers.size();
    //   topological_layer_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //   topological_layer_marker.pose.position.x = 0.0;
    //   topological_layer_marker.pose.position.y = 30.0;
    //   topological_layer_marker.pose.position.z = 12.0;
    //   topological_layer_marker.color.r = color_r;
    //   topological_layer_marker.color.g = color_g;
    //   topological_layer_marker.color.b = color_b;
    //   topological_layer_marker.color.a = 1;
    //   topological_layer_marker.pose.orientation.w = 1.0;
    //   topological_layer_marker.text = "Topological Layer";
    //   markers.markers.push_back(topological_layer_marker);
    // }

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
  ros::WallTimer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  ros::Subscriber cloud_seg_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub;

  ros::Subscriber raw_odom_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;
  ros::Subscriber room_data_sub;

  ros::Publisher map_planes_pub;
  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  bool wait_trans_odom2map, got_trans_odom2map;
  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;
  ros::Publisher odom_pose_corrected_pub;
  ros::Publisher odom_path_corrected_pub;
  std::vector<geometry_msgs::PoseStamped> odom_path_vec;
  ros::Subscriber init_odom2map_sub;

  std::string points_topic;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;

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
  double plane_dist_threshold;
  bool constant_covariance;
  double min_plane_points;
  bool use_point_to_plane;
  bool use_parallel_plane_constraint, use_perpendicular_plane_constraint;
  bool use_corridor_constraint, use_room_constraint;
  double corridor_information;
  double corridor_dist_threshold, corridor_min_plane_length, corridor_min_width, corridor_max_width;
  double corridor_plane_length_diff_threshold, corridor_point_diff_threshold;
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
  enum plane_class : uint8_t {
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
    HORT_PLANE = 2,
  };

  std::mutex vert_plane_snapshot_mutex;
  std::vector<VerticalPlanes> x_vert_planes_snapshot, y_vert_planes_snapshot;  // snapshot of vertically segmented planes

  std::mutex hort_plane_snapshot_mutex;
  std::vector<HorizontalPlanes> hort_planes_snapshot;

  std::mutex corridor_snapshot_mutex;
  std::vector<Corridors> x_corridors_snapshot, y_corridors_snapshot;

  std::mutex room_snapshot_mutex;
  std::vector<Rooms> rooms_vec_snapshot;

  // room data queue
  std::mutex room_data_queue_mutex;
  std::deque<s_graphs::RoomsData> room_data_queue;

  // Seg map queue
  std::mutex cloud_seg_mutex;
  std::deque<s_graphs::PointClouds::Ptr> clouds_seg_queue;

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
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::unique_ptr<PlaneAnalyzer> plane_analyzer;
};

}  // namespace s_graphs

PLUGINLIB_EXPORT_CLASS(s_graphs::SGraphsNodelet, nodelet::Nodelet)
