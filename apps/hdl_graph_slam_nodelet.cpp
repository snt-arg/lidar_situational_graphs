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
#include <hdl_graph_slam/FloorCoeffs.h>
#include <geometry_msgs/PoseStamped.h>

#include <hdl_graph_slam/SaveMap.h>
#include <hdl_graph_slam/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>
#include <hdl_graph_slam/PointClouds.h>


#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/planes.hpp>
#include <hdl_graph_slam/corridors.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

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

namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZRGBNormal PointNormal;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ApproxSyncPolicy;


  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

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

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

    init_odom2map_sub = nh.subscribe("/odom2map/initial_pose", 1, &HdlGraphSlamNodelet::init_map2odom_pose_callback, this);

    while(wait_trans_odom2map && !got_trans_odom2map) {
      ROS_WARN("Waiting for the Initial Transform between odom and map frame");
      ros::spinOnce();
      usleep(1e6);
    }
    

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
  

    imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &HdlGraphSlamNodelet::imu_callback, this);
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &HdlGraphSlamNodelet::floor_coeffs_callback, this);
    cloud_seg_sub = nh.subscribe("/segmented_clouds", 32, &HdlGraphSlamNodelet::cloud_seg_callback, this);

    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this);
    }

    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2map", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1, true);
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);

    dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

    graph_updated = false;
    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }

private:
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
  void cloud_seg_callback(const hdl_graph_slam::PointClouds::Ptr& clouds_seg_msg) {
      std::lock_guard<std::mutex> lock(cloud_seg_mutex);
      clouds_seg_queue.push_back(clouds_seg_msg);
  }

  /** 
  * @brief flush the accumulated cloud seg queue
  */
  bool flush_clouds_seg_queue(){
    std::lock_guard<std::mutex> lock(cloud_seg_mutex);

    if(keyframes.empty() ) {
      std::cout << "No keyframes" << std::endl;  
      return false;
    }
    else if (clouds_seg_queue.empty()) {
      std::cout << "Clouds seg queue is empty" << std::endl;  
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;
   
    bool updated = false; int plane_id;
    for(const auto& clouds_seg_msg : clouds_seg_queue) {
      g2o::Plane3D prev_plane = Eigen::Vector4d(0,0,0,0); int prev_plane_id =-1;
      for(const auto& cloud_seg_msg : clouds_seg_msg->pointclouds) {

        if(cloud_seg_msg.header.stamp > latest_keyframe_stamp) {
          //std::cout << "cloud_seg time is greater than last keyframe stamp" << std::endl;
          break;
        }

        auto found = keyframe_hash.find(cloud_seg_msg.header.stamp);
        if(found == keyframe_hash.end()) {
          continue;
        }

        pcl::PointCloud<PointNormal>::Ptr cloud_seg_body(new pcl::PointCloud<PointNormal>());
        pcl::fromROSMsg(cloud_seg_msg, *cloud_seg_body);

        const auto& keyframe = found->second;
    
        Eigen::Vector4d coeffs_body_frame(cloud_seg_body->back().normal_x, cloud_seg_body->back().normal_y, cloud_seg_body->back().normal_z, cloud_seg_body->back().curvature);          
        g2o::Plane3D det_plane_body_frame = coeffs_body_frame;
        Eigen::Vector4d coeffs_map_frame; Eigen::Isometry3d w2n = keyframe->node->estimate();
        coeffs_map_frame.head<3>() = w2n.rotation() * coeffs_body_frame.head<3>();
        coeffs_map_frame(3) = coeffs_body_frame(3) - w2n.translation().dot(coeffs_map_frame.head<3>());

        int plane_type;
        bool use_point_to_plane = 0;
        int corridor_plane = 0;  
        if (fabs(coeffs_map_frame(0)) > 0.98) {              
          plane_type = plane_class::X_VERT_PLANE; 
          g2o::Plane3D det_plane_map_frame = coeffs_map_frame;
          plane_id = factor_planes(keyframe, cloud_seg_body, det_plane_map_frame, det_plane_body_frame, plane_type, use_point_to_plane);
          updated = true;
        } else if (fabs(coeffs_map_frame(1)) > 0.98) {                   
          plane_type = plane_class::Y_VERT_PLANE;  
          g2o::Plane3D det_plane_map_frame = coeffs_map_frame;
          plane_id = factor_planes(keyframe, cloud_seg_body, det_plane_map_frame, det_plane_body_frame, plane_type, use_point_to_plane);
          if(det_plane_map_frame.coeffs().head(3).dot(prev_plane.coeffs().head(3)) < 0) {
            factor_corridors(plane_type, cloud_seg_body, prev_plane, prev_plane_id, det_plane_map_frame, plane_id);
          }
          prev_plane = det_plane_map_frame;
          prev_plane_id = plane_id;
          updated = true;
        } else if (fabs(coeffs_map_frame(2)) > 0.95) {
          plane_type = plane_class::HORT_PLANE;  
          g2o::Plane3D det_plane_map_frame = coeffs_map_frame;
          plane_id = factor_planes(keyframe, cloud_seg_body, det_plane_map_frame, det_plane_body_frame, plane_type, use_point_to_plane);
          updated = true;
        } else 
          continue;
      }
    }
    auto remove_loc = std::upper_bound(clouds_seg_queue.begin(), clouds_seg_queue.end(), latest_keyframe_stamp, [=](const ros::Time& stamp, const hdl_graph_slam::PointClouds::Ptr& clouds_seg) { return stamp < clouds_seg->header.stamp; });
    clouds_seg_queue.erase(clouds_seg_queue.begin(), remove_loc);

    return updated;
  }
  
  /**  
  *@brief Converting the cloud to map frame
  */
  pcl::PointCloud<PointNormal>::Ptr convert_cloud_to_map(KeyFrame::Ptr keyframe, pcl::PointCloud<PointNormal>::Ptr cloud_seg_body) {
    pcl::PointCloud<PointNormal>::Ptr cloud_seg_map(new pcl::PointCloud<PointNormal>());
    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>();
    for(const auto& src_pt : cloud_seg_body->points) {
      PointNormal dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      cloud_seg_map->push_back(dst_pt);
    }
    return cloud_seg_map;
  }

  /** 
  * @brief create vertical plane factors
  */
  int factor_planes(KeyFrame::Ptr keyframe, pcl::PointCloud<PointNormal>::Ptr cloud_seg_body, g2o::Plane3D det_plane_map_frame, g2o::Plane3D det_plane_body_frame, int plane_type, bool use_point_to_plane) {
    g2o::VertexPlane* plane_node; 
    Eigen::Matrix4d Gij;
    Gij.setZero();  
 
    if(use_point_to_plane) {
      auto it = cloud_seg_body->points.begin();
      while (it != cloud_seg_body->points.end()) {
        PointNormal point_tmp;
        point_tmp = *it;
        Eigen::Vector4d point(point_tmp.x, point_tmp.y, point_tmp.z, 1);
        double point_to_plane_d = det_plane_map_frame.coeffs().transpose() * keyframe->node->estimate().matrix() * point;

        if(abs(point_to_plane_d) < 0.1) {
          Gij += point * point.transpose();
          ++it;
        } else {
          it = cloud_seg_body->points.erase(it);
        } 
      }
    }
      
    pcl::PointCloud<PointNormal>::Ptr cloud_seg_map = convert_cloud_to_map(keyframe, cloud_seg_body);
    if(cloud_seg_map->points.empty()) { 
      std::cout << "Could not convert the cloud to body frame";
      return false;
    }

    std::pair<int,int> data_association; data_association.first = -1;
    bool add_parallel_plane_edge = false, add_perpendicular_plane_edge = false;
    if (plane_type == plane_class::X_VERT_PLANE){  
      data_association = associate_plane(keyframe, det_plane_body_frame.coeffs(), plane_type);
      
      if(x_vert_planes.empty() || data_association.first == -1) {
          data_association.first = graph_slam->num_vertices();
          plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
          //x_vert_plane_node->setFixed(true);
          //std::cout << "Added new x vertical plane node with coeffs " <<  det_plane_map_frame.coeffs() << std::endl;
          VerticalPlanes vert_plane;
          vert_plane.id = data_association.first;
          vert_plane.plane = det_plane_map_frame.coeffs();
          vert_plane.cloud_seg_body = cloud_seg_body;
          vert_plane.cloud_seg_map = cloud_seg_map;
          vert_plane.node = plane_node; 
          vert_plane.covariance = Eigen::Matrix3d::Identity();
          vert_plane.parallel_pair = false;
          x_vert_planes.push_back(vert_plane);
          add_parallel_plane_edge = true; add_perpendicular_plane_edge = true;
      } else {
          //std::cout << "matched x vert plane with x vert plane of id " << std::to_string(data_association.first)  << std::endl;
          plane_node = x_vert_planes[data_association.second].node;
      }
    } else if (plane_type == plane_class::Y_VERT_PLANE) {
      data_association = associate_plane(keyframe, det_plane_body_frame.coeffs(), plane_type);
      
      if(y_vert_planes.empty() || data_association.first == -1) {
        data_association.first = graph_slam->num_vertices();
        plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
        //std::cout << "Added new y vertical plane node with coeffs " <<  det_plane_map_frame.coeffs() << std::endl;
        VerticalPlanes vert_plane;
        vert_plane.id = data_association.first;
        vert_plane.plane = det_plane_map_frame.coeffs();
        vert_plane.cloud_seg_body = cloud_seg_body;
        vert_plane.cloud_seg_map = cloud_seg_map;
        vert_plane.node = plane_node; 
        vert_plane.covariance = Eigen::Matrix3d::Identity();
        vert_plane.parallel_pair = false;
        y_vert_planes.push_back(vert_plane);
        add_parallel_plane_edge = true; add_perpendicular_plane_edge = true;
      } else {
          //std::cout << "matched y vert plane with y vert plane of id " << std::to_string(data_association.first)  << std::endl;
          plane_node = y_vert_planes[data_association.second].node;

        } 
      } else if (plane_type == plane_class::HORT_PLANE) {
        data_association = associate_plane(keyframe, det_plane_body_frame.coeffs(), plane_type);
        
        if(hort_planes.empty() || data_association.first == -1) {
          data_association.first = graph_slam->num_vertices();
          plane_node = graph_slam->add_plane_node(det_plane_map_frame.coeffs());
          //std::cout << "Added new horizontal plane node with coeffs " <<  det_plane_map_frame.coeffs() << std::endl;
          HorizontalPlanes hort_plane;
          hort_plane.id = data_association.first;
          hort_plane.plane = det_plane_map_frame.coeffs();
          hort_plane.cloud_seg_body = cloud_seg_body;
          hort_plane.cloud_seg_map = cloud_seg_map;
          hort_plane.node = plane_node; 
          hort_plane.covariance = Eigen::Matrix3d::Identity();
          hort_planes.push_back(hort_plane);
          add_parallel_plane_edge = true; add_perpendicular_plane_edge = true;
      } else {
        //std::cout << "matched hort plane with hort plane of id " << std::to_string(data_association.first)  << std::endl;
        plane_node = hort_planes[data_association.second].node;
      }
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

    bool use_parallel_constraint = true;
    if(use_parallel_constraint && add_parallel_plane_edge) {
      parallel_plane_constraint(plane_node, data_association.first, plane_type);
    }
    bool use_perpendicular_constraint = true;
    if(use_perpendicular_constraint && add_perpendicular_plane_edge) {
      perpendicular_plane_constraint(plane_node, data_association.first, plane_type);
    }

    keyframe->cloud_seg_body = cloud_seg_body;

    return data_association.first;
  }

  /** 
  * @brief data assoction betweeen the planes
  */
  std::pair<int,int> associate_plane(KeyFrame::Ptr keyframe, g2o::Plane3D det_plane, int plane_type) {
    std::pair<int,int> data_association;
    float min_dist = 100;
    double min_maha_dist = 100;  
    Eigen::Isometry3d m2n = keyframe->estimate().inverse();

    if(plane_type == plane_class::X_VERT_PLANE) {
      for(int i=0; i< x_vert_planes.size(); ++i) { 
        float dist = fabs(det_plane.coeffs()(3) - x_vert_planes[i].plane.coeffs()(3));
        //std::cout << "distance x: " << dist << std::endl;
        if(dist < min_dist){
          min_dist = dist;
          //id = x_vert_planes[i].id;
        }
        g2o::Plane3D local_plane = m2n * x_vert_planes[i].plane;
        Eigen::Vector3d error = local_plane.ominus(det_plane);
        double maha_dist = sqrt(error.transpose() * x_vert_planes[i].covariance.inverse() * error);
        //std::cout << "cov x: " << x_vert_planes[i].covariance.inverse() << std::endl;
        //std::cout << "maha distance x: " << maha_dist << std::endl;

        if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);            
          } 
        if(maha_dist < min_maha_dist) {
          min_maha_dist = maha_dist;
          data_association.first = x_vert_planes[i].id;
          data_association.second = i;
          }
        }
      }

    if(plane_type == plane_class::Y_VERT_PLANE) {
        for(int i=0; i< y_vert_planes.size(); ++i) { 
          float dist = fabs(det_plane.coeffs()(3) - y_vert_planes[i].plane.coeffs()(3));
          //std::cout << "distance y: " << dist << std::endl;
          if(dist < min_dist){
            min_dist = dist;
            //id = y_vert_planes[i].id;
          }
          g2o::Plane3D local_plane = m2n * y_vert_planes[i].plane;
          Eigen::Vector3d error = local_plane.ominus(det_plane);
          double maha_dist = sqrt(error.transpose() * y_vert_planes[i].covariance.inverse() * error);
          //std::cout << "cov y: " << y_vert_planes[i].covariance.inverse() << std::endl;
          //std::cout << "maha distance y: " << maha_dist << std::endl;
          if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);            
          } 
          if(maha_dist < min_maha_dist) {
            min_maha_dist = maha_dist;
            data_association.first = y_vert_planes[i].id;
            data_association.second = i;
            }
          }   
      }

    if(plane_type == plane_class::HORT_PLANE) {
        for(int i=0; i< hort_planes.size(); ++i) { 
          float dist = fabs(det_plane.coeffs()(3) - hort_planes[i].plane.coeffs()(3));
          //std::cout << "distance hort: " << dist << std::endl;
          if(dist < min_dist){
            min_dist = dist;
            //id = y_vert_planes[i].id;
          }
          g2o::Plane3D local_plane = m2n * hort_planes[i].plane;
          Eigen::Vector3d error = local_plane.ominus(det_plane);
          double maha_dist = sqrt(error.transpose() * hort_planes[i].covariance.inverse() * error);
          //std::cout << "cov hor: " << hort_planes[i].covariance.inverse() << std::endl;
          //std::cout << "maha distance hort: " << maha_dist << std::endl;
          if(std::isnan(maha_dist) || maha_dist < 1e-3) {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            maha_dist = sqrt(error.transpose() * cov * error);            
          } 
          if(maha_dist < min_maha_dist) {
            min_maha_dist = maha_dist;
            data_association.first = hort_planes[i].id;
            data_association.second = i;
            }
          }   
      }

      //std::cout << "min_dist: " << min_dist << std::endl;
      //std::cout << "min_mah_dist: " << min_maha_dist << std::endl;

      // if(min_dist > 0.30)
      //   id = -1;
      double threshold;
      if(plane_type == plane_class::HORT_PLANE)
        threshold = 0.15;
      else   
        threshold = 0.15;

      if(min_maha_dist > threshold)
         data_association.first = -1;

    return data_association;
  }
  
  /**  
  * @brief this method add parallel constraint between the planes
  */
  void parallel_plane_constraint(g2o::VertexPlane* plane_node, int id, int plane_type) {
    Eigen::Matrix<double, 1, 1> information(0.001);
    Eigen::Vector3d meas(0,0,0);
    if(plane_type == plane_class::X_VERT_PLANE) {
      for(int i=0; i<x_vert_planes.size(); ++i){
        if(id != x_vert_planes[i].id) {
          auto edge = graph_slam->add_plane_parallel_edge(x_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
          x_vert_planes[i].parallel_pair = true;
        }
      }
    }
    if(plane_type == plane_class::Y_VERT_PLANE) {
      for(int i=0; i<y_vert_planes.size(); ++i){
        if(id != y_vert_planes[i].id) {
          auto edge = graph_slam->add_plane_parallel_edge(y_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
          y_vert_planes[i].parallel_pair = true;
        }
      }
    }
    if(plane_type == plane_class::HORT_PLANE) {
      for(int i=0; i<hort_planes.size(); ++i){
        if(id != hort_planes[i].id) {
          auto edge = graph_slam->add_plane_parallel_edge(hort_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
          hort_planes[i].parallel_pair = true;
        }
      }
    }
  }

  /**  
  * @brief this method adds perpendicular constraint between the planes
  */
  void perpendicular_plane_constraint(g2o::VertexPlane* plane_node, int id, int plane_type) {
    Eigen::Matrix<double, 1, 1> information(0.001);
    Eigen::Vector3d meas(0,0,0);
    if(plane_type == plane_class::X_VERT_PLANE) {
      for(int i=0; i<y_vert_planes.size(); ++i){
          auto edge = graph_slam->add_plane_perpendicular_edge(y_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }
    }
    if(plane_type == plane_class::Y_VERT_PLANE) {
      for(int i=0; i<x_vert_planes.size(); ++i){
          auto edge = graph_slam->add_plane_perpendicular_edge(x_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }
    } 
    if(plane_type == plane_class::HORT_PLANE) {
      for(int i=0; i<x_vert_planes.size(); ++i){
          auto edge = graph_slam->add_plane_perpendicular_edge(x_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }
      for(int i=0; i<y_vert_planes.size(); ++i){
          auto edge = graph_slam->add_plane_perpendicular_edge(y_vert_planes[i].node, plane_node, meas, information);
          graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      }
    } 
  }

  void factor_corridors(int plane_type, pcl::PointCloud<PointNormal>::Ptr cloud_seg, g2o::Plane3D prev_plane, int prev_plane_id, g2o::Plane3D curr_plane, int curr_plane_id) {
    PointNormal pmin, pmax; pcl::PointXY p1, p2;
    pcl::getMaxSegment(*cloud_seg, pmin, pmax);
    p1.x = pmin.x; p1.y = pmin.y;       
    p2.x = pmax.x; p2.y = pmax.y;
    float length = pcl::euclideanDistance(p1,p2);
    std::cout << "length: " << length << std::endl;

    g2o::VertexSE3* corr_node;  std::pair<int,int> corr_data_association;   
    Eigen::Vector3d meas_prev_plane, meas_curr_plane;
    Eigen::Matrix<double, 1, 1> information(0.001);
    Eigen::Isometry3d pose = corridor_pose(prev_plane.coeffs(), curr_plane.coeffs());

    if(plane_type = plane_class::Y_VERT_PLANE) {

      auto found_prev_plane = y_vert_planes.begin();
      auto found_curr_plane = y_vert_planes.begin();
      corr_data_association = associate_corridors(plane_type, pose);

      if(length > 5 && (y_corridors.empty() || corr_data_association.first == -1)) {
        
        std::cout << "found a corridor between plane id " << prev_plane_id << " and plane id " << curr_plane_id << std::endl;
        corr_data_association.first = graph_slam->num_vertices();
        corr_node = graph_slam->add_se3_node(pose);
        corr_node->setFixed(true);
        Corridors det_corridor;
        det_corridor.id = corr_data_association.first;
        det_corridor.plane1 = prev_plane; det_corridor.plane2 = curr_plane; 
        det_corridor.plane1_id = prev_plane_id; det_corridor.plane2_id = curr_plane_id; 
        det_corridor.node = corr_node;      
        y_corridors.push_back(det_corridor);
        
        found_prev_plane = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == prev_plane_id);
        found_curr_plane = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == prev_plane_id);
        meas_prev_plane =  corridor_measurement(pose.translation(), prev_plane.coeffs());
        meas_curr_plane =  corridor_measurement(pose.translation(), prev_plane.coeffs());
        
        auto edge_prev_plane = graph_slam->add_corridor_plane_edge(corr_node, (*found_prev_plane).node, meas_prev_plane, information);
        graph_slam->add_robust_kernel(edge_prev_plane, "Huber", 1.0);

        auto edge_curr_plane = graph_slam->add_corridor_plane_edge(corr_node, (*found_curr_plane).node, meas_curr_plane, information);
        graph_slam->add_robust_kernel(edge_curr_plane, "Huber", 1.0);
      } else {
        /* add the edge between detected planes and the corridor */
        std::cout << "Matched det corridor to mapped corridor with id " << corr_data_association.first << std::endl;
        corr_node = y_corridors[corr_data_association.second].node;
        
        found_prev_plane = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == prev_plane_id);
        found_curr_plane = std::find_if(y_vert_planes.begin(), y_vert_planes.end(), boost::bind(&VerticalPlanes::id, _1) == prev_plane_id);
        meas_prev_plane =  corridor_measurement(pose.translation(), prev_plane.coeffs());
        meas_curr_plane =  corridor_measurement(pose.translation(), prev_plane.coeffs());

        auto edge_prev_plane = graph_slam->add_corridor_plane_edge(corr_node, (*found_prev_plane).node, meas_prev_plane, information);
        graph_slam->add_robust_kernel(edge_prev_plane, "Huber", 1.0);

        auto edge_curr_plane = graph_slam->add_corridor_plane_edge(corr_node, (*found_curr_plane).node, meas_curr_plane, information);
        graph_slam->add_robust_kernel(edge_curr_plane, "Huber", 1.0);
      } 
    }
    
  }

  Eigen::Isometry3d corridor_pose(Eigen::Vector4d v1, Eigen::Vector4d v2) {
    Eigen::Isometry3d corridor_pose;
    corridor_pose.matrix() = Eigen::Matrix4d::Identity();
    if(fabs(v1(3)) > fabs(v2(3))) {
      double size = v1(3) - v2(3);
      corridor_pose.translation()(1) = ((size)/2) + v2(3); 
      corridor_pose.translation()(2) = (size); 
    } else {
      double size = v2(3) - v1(3);
      corridor_pose.translation()(1) = ((size)/2) + v1(3);
      corridor_pose.translation()(2) = (size);  
    }
    return corridor_pose;
  }

  Eigen::Vector3d corridor_measurement(Eigen::Vector3d corr, Eigen::Vector4d plane) {
      Eigen::Vector3d meas(0,0,0);  
      if(fabs(corr(2)) > fabs(plane(3))) {
       meas(0) =  corr(2) - plane(3);
      } else {
       meas(0) =  plane(3) - corr(2);
      }
    return meas;
  }

  std::pair<int,int> associate_corridors(int plane_type, Eigen::Isometry3d corr_pose) {
    float min_dist = 100;
    std::pair<int,int> data_association; data_association.first = -1;
    if(plane_type == plane_class::Y_VERT_PLANE) {
      for(int i=0; i< y_corridors.size(); ++i) { 
        float dist = fabs((corr_pose.translation()(2)) - (y_corridors[i].node->estimate().translation()(2)));
        if(dist < min_dist) {
          min_dist = dist;
          std::cout << "dist: " << dist << std::endl;
          data_association.first = y_corridors[i].id;
          data_association.second = i;
        }
      }
    }

    std::cout << "min dist: " << min_dist << std::endl;
    float threshold = 0.5;
    if (min_dist > threshold) 
      data_association.first = -1;

    return data_association;
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
  void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr& floor_coeffs_msg) {
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

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const ros::Time& stamp, const hdl_graph_slam::FloorCoeffsConstPtr& coeffs) { return stamp < coeffs->header.stamp; });
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

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue() & !flush_clouds_seg_queue()) {
      return;
    }

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
    if((graph_slam->optimize(num_iterations)) > 0)
      compute_plane_cov();

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

    auto markers = create_marker_array(ros::Time::now());
    markers_pub.publish(markers);
  }

  /**  
  * @brief compute the plane covariances
  */
  void compute_plane_cov() {
    g2o::SparseBlockMatrix<Eigen::MatrixXd> plane_spinv_vec;
    std::vector<std::pair<int, int>> plane_pairs_vec;
    for (int i = 0; i < x_vert_planes.size(); ++i) {
      x_vert_planes[i].node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(x_vert_planes[i].node->hessianIndex(), x_vert_planes[i].node->hessianIndex()));
    }
    for (int i = 0; i < y_vert_planes.size(); ++i) {
      y_vert_planes[i].node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(y_vert_planes[i].node->hessianIndex(), y_vert_planes[i].node->hessianIndex()));
    }
    for (int i = 0; i < hort_planes.size(); ++i) {
      hort_planes[i].node->unlockQuadraticForm();
      plane_pairs_vec.push_back(std::make_pair(hort_planes[i].node->hessianIndex(), hort_planes[i].node->hessianIndex()));
    }
    

    if(!plane_pairs_vec.empty()){
      if (graph_slam->compute_landmark_marginals(plane_spinv_vec, plane_pairs_vec)) {
        int i=0;
        while (i < x_vert_planes.size()) {
          //std::cout << "covariance of x plane " << i << " " << y_vert_planes[i].covariance << std::endl;
          x_vert_planes[i].covariance = plane_spinv_vec.block(x_vert_planes[i].node->hessianIndex(), x_vert_planes[i].node->hessianIndex())->eval().cast<double>();
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(x_vert_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
              //std::cout << "covariance of x plane not PSD" << i << " " << x_vert_planes[i].covariance << std::endl;
              x_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++; 
        }
        i=0;
        while (i < y_vert_planes.size()) {
          y_vert_planes[i].covariance = plane_spinv_vec.block(y_vert_planes[i].node->hessianIndex(), y_vert_planes[i].node->hessianIndex())->eval().cast<double>();
          //std::cout << "covariance of y plane " << i << " " << y_vert_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(y_vert_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
              //std::cout << "covariance of y plane not PSD " << i << " " << y_vert_planes[i].covariance << std::endl;
              y_vert_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
       }
       i=0;
       while (i < hort_planes.size()) {
          hort_planes[i].covariance = plane_spinv_vec.block(hort_planes[i].node->hessianIndex(), hort_planes[i].node->hessianIndex())->eval().cast<double>();
          //std::cout << "covariance of y plane " << i << " " << hort_planes[i].covariance << std::endl;
          Eigen::LLT<Eigen::MatrixXd> lltOfCov(hort_planes[i].covariance);
          if(lltOfCov.info() == Eigen::NumericalIssue) {
              //std::cout << "covariance of y plane not PSD " << i << " " << hort_planes[i].covariance << std::endl;
              hort_planes[i].covariance = Eigen::Matrix3d::Identity();
          }
          i++;
       }

      }
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(9);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = map_frame_id;
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker& imu_marker = markers.markers[1];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
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

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = map_frame_id;
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
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

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
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
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2;
        float r=0, g=0, b=0.0;
        double x=0, y=0;
        if (fabs(v2->estimate().normal()(0)) > 0.95) {
          for(auto x_plane : x_vert_planes) {
            if (x_plane.id == v2->id()) {
              x = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].x;
              y = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].y;
            } 
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          r=1.0;
        } 
        else if (fabs(v2->estimate().normal()(1)) > 0.95) {
           for(auto y_plane : y_vert_planes) {
            if (y_plane.id == v2->id()) {
              x = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].x;
              y = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].y;
            } 
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          b=1.0;
        }
        else if (fabs(v2->estimate().normal()(2)) > 0.95) {
           for(auto h_plane : hort_planes) {
            if (h_plane.id == v2->id()) {
              x = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size()/2)].x;
              y = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size()/2)].y;
            }
          } 
          pt2 = Eigen::Vector3d(x, y, 5.0);  
          r=1; g=0.65;
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
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = r;
        edge_marker.colors[i * 2 + 1].g = g;
        edge_marker.colors[i * 2 + 1].b = b;
        edge_marker.colors[i * 2 + 1].a = 1.0;
        continue;
      }

      g2o::EdgeSE3PointToPlane* edge_point_to_plane = dynamic_cast<g2o::EdgeSE3PointToPlane*>(edge);
      if(edge_point_to_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_point_to_plane->vertices()[0]);
        g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_point_to_plane->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2;
        float r=0, g=0, b=0.0;
        double x=0, y=0;
        if (fabs(v2->estimate().normal()(0)) > 0.95) {
          for(auto x_plane : x_vert_planes) {
            if (x_plane.id == v2->id()) {
              x = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].x;
              y = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].y;
            } 
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          r=1.0;
        } 
        else if (fabs(v2->estimate().normal()(1)) > 0.95) {
           for(auto y_plane : y_vert_planes) {
            if (y_plane.id == v2->id()) {
              x = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].x;
              y = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].y;
            } 
          }
          pt2 = Eigen::Vector3d(x, y, 5.0);
          b=1.0;
        }
        else if (fabs(v2->estimate().normal()(2)) > 0.95) {
           for(auto h_plane : hort_planes) {
            if (h_plane.id == v2->id()) {
              x = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size()/2)].x;
              y = h_plane.cloud_seg_map->points[(h_plane.cloud_seg_map->points.size()/2)].y;
            }
          } 
          pt2 = Eigen::Vector3d(x, y, 5.0);  
          r=1; g=0.65;
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
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = r;
        edge_marker.colors[i * 2 + 1].g = g;
        edge_marker.colors[i * 2 + 1].b = b;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      // g2o::EdgePlaneParallel* edge_parallel_plane = dynamic_cast<g2o::EdgePlaneParallel*>(edge);  
      // if(edge_parallel_plane) {
      //   g2o::VertexPlane* v1 = dynamic_cast<g2o::VertexPlane*>(edge_parallel_plane->vertices()[0]);
      //   g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_parallel_plane->vertices()[1]);
      //   Eigen::Vector3d pt1(0,0,0), pt2(0,0,0);
      //   float r=0, g=0, b=0.0;
      //   double x1=0, y1=0, x2 =0, y2=0;
      //   if (fabs(v2->estimate().normal()(0)) > 0.95) {
      //     for(auto x_plane : x_vert_planes) {
      //       if (x_plane.id == v1->id()) {
      //         x1 = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].x;
      //         y1 = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].y;
      //       } else if(x_plane.id == v2->id()) {
      //         x2 = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].x;
      //         y2 = x_plane.cloud_seg_map->points[(x_plane.cloud_seg_map->points.size()/2)].y;
      //       } 
      //     }
      //     pt1 = Eigen::Vector3d(x1, y1, 10.0);
      //     pt2 = Eigen::Vector3d(x2, y2, 10.0);
      //   }
      //   if (fabs(v2->estimate().normal()(1)) > 0.95) {
      //     for(auto y_plane : y_vert_planes) {
      //       if (y_plane.id == v1->id()) {
      //         x1 = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].x;
      //         y1 = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].y;
      //       } else if(y_plane.id == v2->id()) {
      //         x2 = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].x;
      //         y2 = y_plane.cloud_seg_map->points[(y_plane.cloud_seg_map->points.size()/2)].y;
      //       } 
      //     }
      //     pt1 = Eigen::Vector3d(x1, y1, 10.0);
      //     pt2 = Eigen::Vector3d(x2, y2, 10.0);
      //   }
        
      //   edge_marker.points[i * 2].x = pt1.x();
      //   edge_marker.points[i * 2].y = pt1.y();
      //   edge_marker.points[i * 2].z = pt1.z();
      //   edge_marker.points[i * 2 + 1].x = pt2.x();
      //   edge_marker.points[i * 2 + 1].y = pt2.y();
      //   edge_marker.points[i * 2 + 1].z = pt2.z();

      //   edge_marker.colors[i * 2].r = r;
      //   edge_marker.colors[i * 2].g = g;
      //   edge_marker.colors[i * 2].b = b;
      //   edge_marker.colors[i * 2].a = 1.0;
      //   edge_marker.colors[i * 2 + 1].r = r;
      //   edge_marker.colors[i * 2 + 1].g = g;
      //   edge_marker.colors[i * 2 + 1].b = b;
      //   edge_marker.colors[i * 2 + 1].a = 1.0;

      //   continue;
      // }

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

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = map_frame_id;
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
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

    //x vertical plane markers 
    visualization_msgs::Marker& x_vert_plane_marker = markers.markers[4];
    x_vert_plane_marker.pose.orientation.w = 1.0;
    x_vert_plane_marker.scale.x = 0.05;
    x_vert_plane_marker.scale.y = 0.05;
    x_vert_plane_marker.scale.z = 0.05;
    //plane_marker.points.resize(vert_planes.size());    
    x_vert_plane_marker.header.frame_id = map_frame_id;
    x_vert_plane_marker.header.stamp = stamp;
    x_vert_plane_marker.ns = "x_vert_planes";
    x_vert_plane_marker.id = 4;
    x_vert_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;

    for(int i = 0; i < x_vert_planes.size(); ++i) {
      for(size_t j=0; j < x_vert_planes[i].cloud_seg_map->size(); ++j) {
        geometry_msgs::Point point;
        point.x = x_vert_planes[i].cloud_seg_map->points[j].x;
        point.y = x_vert_planes[i].cloud_seg_map->points[j].y;
        point.z = x_vert_planes[i].cloud_seg_map->points[j].z + 5.0;
        x_vert_plane_marker.points.push_back(point);
        // if (x_vert_planes[i].parallel_pair) {
        //   geometry_msgs::Point parallel_plane_point;
        //   parallel_plane_point.x = x_vert_planes[i].cloud_seg_map->points[j].x;
        //   parallel_plane_point.y = x_vert_planes[i].cloud_seg_map->points[j].y;
        //   parallel_plane_point.z = x_vert_planes[i].cloud_seg_map->points[j].z + 10.0;
        //   x_vert_plane_marker.points.push_back(parallel_plane_point);
        // }
      }
      x_vert_plane_marker.color.r = 1;
      x_vert_plane_marker.color.a = 1;
    }

    //y vertical plane markers 
    visualization_msgs::Marker& y_vert_plane_marker = markers.markers[5];
    y_vert_plane_marker.pose.orientation.w = 1.0;
    y_vert_plane_marker.scale.x = 0.05;
    y_vert_plane_marker.scale.y = 0.05;
    y_vert_plane_marker.scale.z = 0.05;
    //plane_marker.points.resize(vert_planes.size());    
    y_vert_plane_marker.header.frame_id = map_frame_id;
    y_vert_plane_marker.header.stamp = stamp;
    y_vert_plane_marker.ns = "y_vert_planes";
    y_vert_plane_marker.id = 5;
    y_vert_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;
   
    for(int i = 0; i < y_vert_planes.size(); ++i) {
      for(size_t j=0; j < y_vert_planes[i].cloud_seg_map->size(); ++j) { 
        geometry_msgs::Point point;
        point.x = y_vert_planes[i].cloud_seg_map->points[j].x;
        point.y = y_vert_planes[i].cloud_seg_map->points[j].y;
        point.z = y_vert_planes[i].cloud_seg_map->points[j].z + 5.0;
        y_vert_plane_marker.points.push_back(point);
        // if (y_vert_planes[i].parallel_pair) {
        //   geometry_msgs::Point parallel_plane_point;
        //   parallel_plane_point.x = y_vert_planes[i].cloud_seg_map->points[j].x;
        //   parallel_plane_point.y = y_vert_planes[i].cloud_seg_map->points[j].y;
        //   parallel_plane_point.z = y_vert_planes[i].cloud_seg_map->points[j].z + 10.0;
        //   y_vert_plane_marker.points.push_back(parallel_plane_point);
        // }
      }
      y_vert_plane_marker.color.b = 1;
      y_vert_plane_marker.color.a = 1;
    }

    //horizontal plane markers 
    visualization_msgs::Marker& hort_plane_marker = markers.markers[6];
    hort_plane_marker.pose.orientation.w = 1.0;
    hort_plane_marker.scale.x = 0.05;
    hort_plane_marker.scale.y = 0.05;
    hort_plane_marker.scale.z = 0.05;
    //plane_marker.points.resize(vert_planes.size());    
    hort_plane_marker.header.frame_id = map_frame_id;
    hort_plane_marker.header.stamp = stamp;
    hort_plane_marker.ns = "hort_planes";
    hort_plane_marker.id = 6;
    hort_plane_marker.type = visualization_msgs::Marker::CUBE_LIST;
   
    for(int i = 0; i < hort_planes.size(); ++i) {
      for(size_t j=0; j < hort_planes[i].cloud_seg_map->size(); ++j) { 
        geometry_msgs::Point point;
        point.x = hort_planes[i].cloud_seg_map->points[j].x;
        point.y = hort_planes[i].cloud_seg_map->points[j].y;
        point.z = hort_planes[i].cloud_seg_map->points[j].z + 5.0;
        hort_plane_marker.points.push_back(point);
      }
      hort_plane_marker.color.r = 1;
      hort_plane_marker.color.g = 0.65;
      hort_plane_marker.color.a = 1;
    }

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(hdl_graph_slam::DumpGraphRequest& req, hdl_graph_slam::DumpGraphResponse& res) {
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
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
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

  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  bool wait_trans_odom2map, got_trans_odom2map;
  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;
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
  std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

  //vertical and horizontal planes
  std::vector<VerticalPlanes> x_vert_planes, y_vert_planes;         // vertically segmented planes
  std::vector<HorizontalPlanes> hort_planes;      // horizontally segmented planes
  std::vector<Corridors> y_corridors;
  enum plane_class : uint8_t{
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
    HORT_PLANE = 2,
  };

  // Seg map queue
  std::mutex cloud_seg_mutex;
  std::deque<hdl_graph_slam::PointClouds::Ptr> clouds_seg_queue; 

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;
  
  
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
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
