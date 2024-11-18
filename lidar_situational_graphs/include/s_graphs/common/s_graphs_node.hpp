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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <ctime>
#include <mutex>
#include <rclcpp/time.hpp>
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
#include <s_graphs/common/point_types.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/rooms.hpp>
#include <s_graphs/common/ros_time_hash.hpp>
#include <s_graphs/common/ros_utils.hpp>
#include <s_graphs/common/walls.hpp>
#include <s_graphs/frontend/keyframe_updater.hpp>
#include <s_graphs/frontend/loop_detector.hpp>
#include <s_graphs/frontend/plane_analyzer.hpp>
#include <s_graphs/visualization/graph_publisher.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "situational_graphs_msgs/msg/floor_data.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/point_clouds.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "situational_graphs_msgs/msg/wall_data.hpp"
#include "situational_graphs_msgs/msg/walls_data.hpp"
#include "situational_graphs_msgs/srv/dump_graph.hpp"
#include "situational_graphs_msgs/srv/load_graph.hpp"
#include "situational_graphs_msgs/srv/save_map.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.h"

namespace s_graphs {

class SGraphsNode : public rclcpp::Node {
 public:
  SGraphsNode();
  ~SGraphsNode() {}

 public:
  void start_timers();

 private:
  void declare_ros_params();
  void init_subclass();

 private:
  /**
   * @brief receive the raw odom msg to publish the corrected odom after
   *
   */
  void raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  /**
   * @brief receive the initial transform between map and odom frame
   * @param map2odom_pose_msg
   */
  void init_map2odom_pose_callback(
      geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg,
                      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  /**
   * @brief get the room data from room segmentation module
   *
   */
  void room_data_callback(
      const situational_graphs_msgs::msg::RoomsData::SharedPtr rooms_msg);
  /**
   * @brief get the floor data from floor segmentation module
   *
   */
  void floor_data_callback(
      const situational_graphs_msgs::msg::FloorData::SharedPtr floor_data_msg);
  /**
   * @brief
   *
   * @param nmea_msg
   */
  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr nmea_msg);
  /**
   * @brief
   *
   * @param navsat_msg
   */
  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr navsat_msg);
  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr gps_msg);
  /**
   * @brief
   *
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  /**
   * @brief
   *
   * @param walls_msg
   */
  void wall_data_callback(
      const situational_graphs_msgs::msg::WallsData::SharedPtr walls_msg);

 private:
  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph
   * (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue();
  /**
   * @brief flush the room data from room data queue
   *
   */
  void flush_room_data_queue();
  /**
   * @brief Set the duplicate planes object
   *
   * @param dupl_x_vert_planes
   * @param dupl_y_vert_planes
   */
  void set_duplicate_planes(
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_y_vert_planes);
  /**
   * @brief
   *
   */
  void flush_floor_data_queue();
  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool flush_gps_queue();
  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool flush_imu_queue();
  /**
   * @brief
   *
   * @param stair_keyframe_ids
   */
  void add_stair_keyframes_to_floor(const std::vector<int>& stair_keyframe_ids);
  /**
   * @brief
   *
   */
  void add_first_floor_node();
  /**
   * @brief
   *
   * @param pose
   */
  void update_first_floor_node(const Eigen::Isometry3d& pose);
  /**
   *@brief extract all the keyframes from the found room
   **/
  void extract_keyframes_from_room(Rooms& current_room);

 private:
  /**
   * @brief this methods adds all the data in the queues to the pose graph
   * @param event
   */
  void keyframe_update_timer_callback();
  /**
   * @brief this methods optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback();
  /**
   * @brief
   *
   */
  void handle_global_optimization();
  /**
   * @brief
   *
   */
  void handle_local_global_optimization();
  /**
   * @brief
   *
   */
  void handle_floor_global_optimization();
  /**
   * @brief optimize the local room graphs
   *
   * @param room_id
   * @param num_iterations
   */
  void broadcast_room_graph(const int room_id, const int num_iterations);
  /**
   * @brief
   *
   * @param kf_snapshot
   * @param x_planes_snapshot
   * @param y_planes_snapshot
   * @param hort_planes_snapshot
   * @param x_inf_rooms_snapshot
   * @param y_inf_rooms_snapshot
   * @param rooms_vec_snapshot
   * @param floors_vec_snapshot
   */
  void copy_data(std::vector<KeyFrame::Ptr>& kf_snapshot,
                 std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
                 std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
                 std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
                 std::unordered_map<int, InfiniteRooms>& x_inf_rooms_snapshot,
                 std::unordered_map<int, InfiniteRooms>& y_inf_rooms_snapshot,
                 std::unordered_map<int, Rooms>& rooms_vec_snapshot,
                 std::map<int, Floors>& floors_vec_snapshot);

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_publish_timer_callback(bool pass);

  /**
   * @brief
   *
   * @param current_time
   * @param floor_level
   * @param odom_cloud_queue
   * @param floors_vec_snapshot
   * @param s_graphs_cloud_msg
   */
  void handle_map_cloud(
      const rclcpp::Time& current_time,
      int floor_level,
      std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>>&
          odom_cloud_queue,
      std::map<int, Floors> floors_vec_snapshot,
      sensor_msgs::msg::PointCloud2& s_graphs_cloud_msg);

  /**
   * @brief
   *
   * @param current_time
   * @param floor_level
   * @param is_optimization_global
   * @param prev_mapped_kfs
   * @param kf_snapshot
   * @param floors_vec_snapshot
   * @param s_graphs_cloud_msg
   * @return * void
   */
  void handle_map_cloud(const rclcpp::Time& current_time,
                        int floor_level,
                        bool is_optimization_global,
                        int prev_mapped_kfs,
                        const std::vector<KeyFrame::Ptr>& kf_snapshot,
                        std::map<int, Floors>& floors_vec_snapshot,
                        sensor_msgs::msg::PointCloud2& s_graphs_cloud_msg);

  /**
   * @brief Get the floor map transform object
   *
   * @param floor
   * @return Eigen::Matrix4f
   */
  Eigen::Matrix4f get_floor_map_transform(const Floors floor);

  /**
   * @brief
   *
   * @param floor_level
   * @param x_planes_snapshot
   * @param y_planes_snapshot
   * @param hort_planes_snapshot
   * @param floors_vec_snapshot
   * @param floor_wall_cloud_msg
   */
  void handle_floor_wall_cloud(
      const int& floor_level,
      const std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
      const std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
      const std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
      std::map<int, Floors>& floors_vec_snapshot,
      sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg);

  /**
   * @brief
   *
   * @param current_time
   * @param floor_level
   * @param floors_vec_snapshot
   */
  void concatenate_floor_wall_clouds(
      const int& floor_level,
      sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg,
      const std::map<int, Floors>& floors_vec_snapshot);

  /**
   * @brief
   *
   * @param current_time
   * @param floor_level
   * @param prev_mapped_kfs
   * @param kf_snapshot
   * @param x_planes_snapshot
   * @param y_planes_snapshot
   * @param hort_planes_snapshot
   * @param floor_wall_cloud_msg
   */
  void handle_floor_wall_cloud(
      const rclcpp::Time& current_time,
      const int& floor_level,
      const int& prev_mapped_kfs,
      const std::vector<KeyFrame::Ptr>& kf_snapshot,
      const std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
      const std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
      const std::unordered_map<int, HorizontalPlanes>& hort_planes_snapshot,
      sensor_msgs::msg::PointCloud2& floor_wall_cloud_msg);

  /**
   * @brief
   *
   * @param kf
   * @param floor_level
   * @param plane
   * @param wall_map_cloud
   * @return * template <typename planeT>
   */
  template <typename planeT>
  void update_wall_cloud(const KeyFrame::Ptr kf,
                         const int& floor_level,
                         const planeT plane,
                         pcl::PointCloud<PointNormal>::Ptr& wall_map_cloud);
  /**
   * @brief
   *
   * @param local_covisibility_graph
   * @param keyframes_complete_snapshot
   * @param x_planes_snapshot
   * @param y_planes_snapshot
   * @param x_inf_rooms_snapshot
   * @param y_inf_rooms_snapshot
   * @param rooms_vec_snapshot
   */
  void publish_graph(g2o::SparseOptimizer* local_covisibility_graph,
                     std::map<int, KeyFrame::Ptr> keyframes_complete_snapshot,
                     std::unordered_map<int, VerticalPlanes>& x_planes_snapshot,
                     std::unordered_map<int, VerticalPlanes>& y_planes_snapshot,
                     std::unordered_map<int, InfiniteRooms>& x_inf_rooms_snapshot,
                     std::unordered_map<int, InfiniteRooms>& y_inf_rooms_snapshot,
                     std::unordered_map<int, Rooms>& rooms_vec_snapshot);
  /**
   * @brief
   *
   * @param input_floor_cloud
   * @param target_frame_id
   * @param source_frame_id
   * @return sensor_msgs::msg::PointCloud2
   */
  sensor_msgs::msg::PointCloud2 transform_floor_cloud(
      const sensor_msgs::msg::PointCloud2 input_floor_cloud,
      const std::string target_frame_id,
      const std::string source_frame_id);

  /**
   * @brief
   *
   * @param input_floor_cloud
   * @param transform_stamped
   * @return sensor_msgs::msg::PointCloud2
   */
  inline sensor_msgs::msg::PointCloud2 apply_transform(
      const sensor_msgs::msg::PointCloud2 input_floor_cloud,
      const geometry_msgs::msg::TransformStamped transform_stamped);

  /**
   * @brief publish the mapped plane information from the last n keyframes
   *
   */
  void publish_mapped_planes(
      std::unordered_map<int, VerticalPlanes> x_vert_planes_snapshot,
      std::unordered_map<int, VerticalPlanes> y_vert_planes_snapshot);

  /**
   * @brief publish all the mapped plane information from the entire set of keyframes
   *
   */
  void publish_all_mapped_planes(
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes_snapshot,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes_snapshot);

  /**
   * @brief publish odom corrected pose and path
   */
  void publish_corrected_odom(geometry_msgs::msg::PoseStamped pose_stamped_corrected);

  /**
   * @brief publish static transforms between map and floor levels
   *
   */
  void publish_static_tfs();

  /**
   * @brief Set the dump directory object
   *
   */
  void set_dump_directory();

  /**
   * @brief
   *
   * @param old_directory
   * @param new_destination
   * @return std::string
   */
  std::string move_directory_to_new_destination(const std::string& old_directory,
                                                const std::string& new_destination);

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(
      const std::shared_ptr<situational_graphs_msgs::srv::DumpGraph::Request> req,
      std::shared_ptr<situational_graphs_msgs::srv::DumpGraph::Response> res);

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(
      const std::shared_ptr<situational_graphs_msgs::srv::SaveMap::Request> req,
      std::shared_ptr<situational_graphs_msgs::srv::SaveMap::Response> res);

  /**
   * @brief load all data from the current directory
   * @param req
   * @param res
   * @return
   */
  bool load_service(
      const std::shared_ptr<situational_graphs_msgs::srv::LoadGraph::Request> req,
      std::shared_ptr<situational_graphs_msgs::srv::LoadGraph::Response> res);

  /**
   * @brief
   *
   * @param directories
   */
  void sort_directories(std::vector<std::string>& directories);

 private:
  // ROS
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::TimerBase::SharedPtr optimization_timer;
  rclcpp::TimerBase::SharedPtr keyframe_timer;
  rclcpp::TimerBase::SharedPtr map_publish_timer;
  rclcpp::TimerBase::SharedPtr static_tf_timer;

  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::CallbackGroup::SharedPtr callback_group_publisher;

  rclcpp::CallbackGroup::SharedPtr callback_group_opt_timer;
  rclcpp::CallbackGroup::SharedPtr callback_keyframe_timer;
  rclcpp::CallbackGroup::SharedPtr callback_map_pub_timer;
  rclcpp::CallbackGroup::SharedPtr callback_static_tf_timer;

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
  rclcpp::Subscription<situational_graphs_msgs::msg::RoomsData>::SharedPtr
      room_data_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::WallsData>::SharedPtr
      wall_data_sub;
  rclcpp::Subscription<situational_graphs_msgs::msg::FloorData>::SharedPtr
      floor_data_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_odom2map_sub,
      map_2map_transform_sub;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  bool wait_trans_odom2map, got_trans_odom2map;
  std::vector<geometry_msgs::msg::PoseStamped> odom_path_vec;
  std::string map_frame_id;
  std::string odom_frame_id;
  std::string base_frame_id;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom2map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pose_corrected_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_corrected_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_points_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr map_planes_pub;
  rclcpp::Publisher<situational_graphs_msgs::msg::PlanesData>::SharedPtr
      all_map_planes_pub;
  rclcpp::Publisher<situational_graphs_reasoning_msgs::msg::Graph>::SharedPtr graph_pub;
  rclcpp::Publisher<situational_graphs_reasoning_msgs::msg::GraphKeyframes>::SharedPtr
      graph_keyframes_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom2map_broadcaster;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> floor_map_static_transforms;

  rclcpp::Service<situational_graphs_msgs::srv::DumpGraph>::SharedPtr
      dump_service_server;
  rclcpp::Service<situational_graphs_msgs::srv::LoadGraph>::SharedPtr
      load_service_server;
  rclcpp::Service<situational_graphs_msgs::srv::SaveMap>::SharedPtr
      save_map_service_server;

  // keyframe queue
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  std::mutex odom_cloud_queue_mutex;
  std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<PointT>::Ptr>> odom_cloud_queue;

  // gps queue
  int gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Vector3d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr> gps_queue;

  // imu queue
  int imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue;

  std::deque<int> room_local_graph_id_queue;
  int optimization_window_size;
  bool loop_found, duplicate_planes_found;
  bool global_optimization;
  size_t keyframe_window_size;
  bool extract_planar_surfaces;
  bool constant_covariance;
  double min_plane_points;
  double infinite_room_information;
  double room_information, plane_information;
  bool on_stairs;
  bool floor_node_updated;
  double floor_level_viz_height, keyframe_viz_height, wall_viz_height, room_viz_height,
      floor_node_viz_height;

  enum optimization_class : uint8_t {
    GLOBAL = 1,
    FLOOR_GLOBAL = 2,
    LOCAL_GLOBAL = 3,
  } ongoing_optimization_class;

  // vertical and horizontal planes
  std::unordered_map<int, VerticalPlanes> x_vert_planes, y_vert_planes,
      x_vert_planes_prior, y_vert_planes_prior;  // vertically segmented planes
  std::deque<std::pair<VerticalPlanes, VerticalPlanes>> dupl_x_vert_planes,
      dupl_y_vert_planes;  // vertically segmented planes
  std::unordered_map<int, HorizontalPlanes>
      hort_planes;  // horizontally segmented planes
  std::unordered_map<int, InfiniteRooms> x_infinite_rooms,
      y_infinite_rooms;  // infinite_rooms segmented from planes
  std::unordered_map<int, Rooms> rooms_vec,
      rooms_vec_prior;  // rooms segmented from planes
  std::map<int, Floors> floors_vec;
  std::unordered_map<int, Walls> walls_vec;
  int prev_edge_count, curr_edge_count;

  // room data queue
  std::mutex room_data_queue_mutex, floor_data_mutex;
  std::deque<situational_graphs_msgs::msg::RoomsData> room_data_queue;
  std::deque<situational_graphs_msgs::msg::FloorData> floor_data_queue;

  // for map cloud generation
  bool fast_mapping, viz_dense_map, save_dense_map, viz_all_floor_cloud;
  double map_cloud_resolution;
  double map_cloud_pub_resolution;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  std::mutex graph_mutex;
  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  std::map<int, KeyFrame::Ptr> keyframes;
  std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;
  int current_floor_level, current_session_id;

  int prev_mapped_keyframes;
  visualization_msgs::msg::MarkerArray s_graphs_markers;

  std::string dump_directory;

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