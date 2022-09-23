#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <s_graphs/PointClouds.h>
#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>
#include <s_graphs/PlaneData.h>
#include <s_graphs/PlanesData.h>
#include <s_graphs/room_analyzer.hpp>
#include <s_graphs/plane_utils.hpp>
#include <s_graphs/floor_analyzer.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

namespace s_graphs {

class FloorPlanNodelet : public nodelet::Nodelet {
public:
  FloorPlanNodelet() {}
  virtual ~FloorPlanNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    this->initialize_params();
    this->init_ros();
  }

private:
  void initialize_params() {
    plane_utils.reset(new PlaneUtils());
    room_analyzer.reset(new RoomAnalyzer(private_nh, plane_utils));
    floor_analyzer.reset(new FloorAnalyzer(private_nh, plane_utils));
  }

  void init_ros() {
    skeleton_graph_sub = nh.subscribe("/voxblox_skeletonizer/sparse_graph", 1, &FloorPlanNodelet::skeleton_graph_callback, this);
    map_planes_sub = nh.subscribe("/s_graphs/all_map_planes", 100, &FloorPlanNodelet::map_planes_callback, this);

    floor_plane_timer = nh.createTimer(ros::Duration(10.0), &FloorPlanNodelet::floor_plan_callback, this);
    all_rooms_data_pub = nh.advertise<s_graphs::RoomsData>("/floor_plan/all_rooms_data", 1, false);
    floor_data_pub = nh.advertise<s_graphs::RoomData>("/floor_plan/floor_data", 1, false);
  }

  template<typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if(find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room candidates
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
    room_analyzer->analyze_skeleton_graph(skeleton_graph_msg);
  }

  /**
   * @brief get all the mapped planes from all the keyframes
   *
   */
  void map_planes_callback(const s_graphs::PlanesData::Ptr& map_planes_msg) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    x_vert_plane_queue.push_back(map_planes_msg->x_planes);
    y_vert_plane_queue.push_back(map_planes_msg->y_planes);
  }

  void flush_map_planes(std::vector<s_graphs::PlaneData>& current_x_vert_planes, std::vector<s_graphs::PlaneData>& current_y_vert_planes) {
    std::lock_guard<std::mutex> lock(map_plane_mutex);
    for(const auto& x_map_planes_msg : x_vert_plane_queue) {
      for(const auto& x_map_plane : x_map_planes_msg) {
        if(!contains(current_x_vert_planes, x_map_plane) || current_x_vert_planes.empty()) {
          current_x_vert_planes.push_back(x_map_plane);
        } else {
          continue;
        }
      }
      x_vert_plane_queue.pop_front();
    }

    for(const auto& y_map_planes_msg : y_vert_plane_queue) {
      for(const auto& y_map_plane : y_map_planes_msg) {
        if(!contains(current_y_vert_planes, y_map_plane) || current_y_vert_planes.empty()) {
          current_y_vert_planes.push_back(y_map_plane);
        } else {
          continue;
        }
      }
      y_vert_plane_queue.pop_front();
    }
  }

  void floor_plan_callback(const ros::TimerEvent& event) {
    std::vector<s_graphs::PlaneData> current_x_vert_planes, current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if(current_x_vert_planes.empty() && current_y_vert_planes.empty()) {
      std::cout << "Did not receive any mapped planes" << std::endl;
      return;
    }

    auto t1 = ros::Time::now();
    // extract_rooms(current_x_vert_planes, current_y_vert_planes);
    extract_floors(current_x_vert_planes, current_y_vert_planes);
    auto t2 = ros::Time::now();
    std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 - t1).toSec() << std::endl;
  }

  void extract_rooms(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes) {
    int room_cluster_counter = 0;
    std::vector<s_graphs::RoomData> room_candidates_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters = room_analyzer->get_cloud_clusters();
    std::vector<std::pair<int, int>> connected_subgraph_map = room_analyzer->get_connected_graph();

    for(const auto& cloud_cluster : curr_cloud_clusters) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);

      if(cloud_cluster->points.size() < 10) continue;

      float hull_area;
      room_analyzer->get_convex_hull(cloud_cluster, cloud_hull, hull_area);
      if(hull_area < 1.5) {
        std::cout << "subgraph area too small to be a room " << std::endl;
        continue;
      }

      room_analyzer->perform_room_segmentation(current_x_vert_planes, current_y_vert_planes, room_cluster_counter, cloud_cluster, cloud_hull, room_candidates_vec, connected_subgraph_map);

      s_graphs::RoomsData room_candidates_msg;
      room_candidates_msg.header.stamp = ros::Time::now();
      room_candidates_msg.rooms = room_candidates_vec;
      all_rooms_data_pub.publish(room_candidates_msg);
    }
  }

  void extract_floors(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes) {
    std::vector<s_graphs::PlaneData> floor_plane_candidates_vec;
    floor_analyzer->perform_floor_segmentation(current_x_vert_planes, current_y_vert_planes, floor_plane_candidates_vec);

    geometry_msgs::Point floor_center;
    if(floor_plane_candidates_vec.size() == 4) {
      floor_center = room_analyzer->get_room_center(floor_plane_candidates_vec[0], floor_plane_candidates_vec[1], floor_plane_candidates_vec[2], floor_plane_candidates_vec[3]);

      s_graphs::RoomData floor_data_msg;
      floor_data_msg.header.stamp = ros::Time::now();
      floor_data_msg.id = 0;
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[0]);
      floor_data_msg.x_planes.push_back(floor_plane_candidates_vec[1]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[2]);
      floor_data_msg.y_planes.push_back(floor_plane_candidates_vec[3]);
      floor_data_msg.room_center = floor_center;
      floor_data_pub.publish(floor_data_msg);
    }
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::Subscriber skeleton_graph_sub;
  ros::Subscriber map_planes_sub;
  ros::Publisher all_rooms_data_pub;
  ros::Publisher floor_data_pub;

private:
  ros::Timer floor_plane_timer;

private:
  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::unique_ptr<FloorAnalyzer> floor_analyzer;
  std::shared_ptr<PlaneUtils> plane_utils;

  std::mutex map_plane_mutex;
  std::deque<std::vector<s_graphs::PlaneData>> x_vert_plane_queue, y_vert_plane_queue;
};

}  // namespace s_graphs
PLUGINLIB_EXPORT_CLASS(s_graphs::FloorPlanNodelet, nodelet::Nodelet)
