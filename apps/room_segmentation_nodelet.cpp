#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <boost/format.hpp>

#include <s_graphs/room_analyzer.hpp>
#include <s_graphs/plane_utils.hpp>

#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ml/kmeans.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

namespace s_graphs {

class RoomSegmentationNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZRGBNormal PointT;

  RoomSegmentationNodelet() {}
  virtual ~RoomSegmentationNodelet() {}

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
  }

  void init_ros() {
    skeleton_graph_sub = nh.subscribe("/voxblox_skeletonizer/sparse_graph", 1, &RoomSegmentationNodelet::skeleton_graph_callback, this);
    map_planes_sub = nh.subscribe("/s_graphs/map_planes", 100, &RoomSegmentationNodelet::map_planes_callback, this);

    cluster_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_cloud", 1, true);
    cluster_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_clouds", 1, true);
    room_data_pub = nh.advertise<s_graphs::RoomsData>("/room_segmentation/room_data", 1, false);
    room_centers_pub = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/room_centers", 1, true);
    refined_skeleton_graph_pub = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/refined_skeleton_graph", 1, true);

    room_detection_timer = nh.createTimer(ros::Duration(1.0), &RoomSegmentationNodelet::room_detection_callback, this);
  }

  template<typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if(find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  void room_detection_callback(const ros::TimerEvent& event) {
    std::vector<s_graphs::PlaneData> current_x_vert_planes, current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if(current_x_vert_planes.empty() && current_y_vert_planes.empty()) {
      std::cout << "Did not receive any mapped planes" << std::endl;
      return;
    }

    auto t1 = ros::Time::now();
    extract_rooms(current_x_vert_planes, current_y_vert_planes);
    auto t2 = ros::Time::now();
    std::cout << "duration to extract clusters: " << boost::format("%.3f") % (t2 - t1).toSec() << std::endl;
  }

  /**
   * @brief get the vertical planes in map frame
   * @param map_planes_msg
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

  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room candidates
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
    room_analyzer->analyze_skeleton_graph(skeleton_graph_msg);
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud
   *
   */
  void extract_rooms(std::vector<s_graphs::PlaneData> current_x_vert_planes, std::vector<s_graphs::PlaneData> current_y_vert_planes) {
    int room_cluster_counter = 0;
    visualization_msgs::MarkerArray refined_skeleton_marker_array;
    std::vector<s_graphs::RoomData> room_candidates_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visualizer(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_visualizer(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters = room_analyzer->get_cloud_clusters();
    std::vector<std::pair<int, int>> connected_subgraph_map = room_analyzer->get_connected_graph();
    visualization_msgs::MarkerArray skeleton_marker_array = room_analyzer->get_makerarray_clusters();

    int cluster_id = 0;
    for(const auto& cloud_cluster : curr_cloud_clusters) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);

      if(cloud_cluster->points.size() < 10) continue;

      float hull_area;
      room_analyzer->get_convex_hull(cloud_cluster, cloud_hull, hull_area);
      if(hull_area < 1.5) {
        std::cout << "subgraph area too small to be a room " << std::endl;
        continue;
      }

      bool found_room = room_analyzer->perform_room_segmentation(current_x_vert_planes, current_y_vert_planes, room_cluster_counter, cloud_cluster, cloud_hull, room_candidates_vec, connected_subgraph_map);
      if(found_room) {
        refined_skeleton_marker_array.markers.push_back(skeleton_marker_array.markers[2 * cluster_id]);
        refined_skeleton_marker_array.markers.push_back(skeleton_marker_array.markers[2 * cluster_id + 1]);
      }
      for(int i = 0; i < cloud_cluster->points.size(); ++i) {
        cloud_visualizer->points.push_back(cloud_cluster->points[i]);
      }
      cluster_id++;
    }

    s_graphs::RoomsData room_candidates_msg;
    room_candidates_msg.header.stamp = ros::Time::now();
    room_candidates_msg.rooms = room_candidates_vec;
    room_data_pub.publish(room_candidates_msg);
    viz_room_centers(room_candidates_msg);

    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_visualizer, cloud_cluster_msg);
    cloud_cluster_msg.header.stamp = ros::Time::now();
    cloud_cluster_msg.header.frame_id = "map";
    cluster_cloud_pub.publish(cloud_cluster_msg);

    refined_skeleton_graph_pub.publish(refined_skeleton_marker_array);
  }

  void viz_room_centers(s_graphs::RoomsData room_vec) {
    visualization_msgs::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = "map";
    room_marker.header.stamp = ros::Time::now();
    room_marker.ns = "rooms";
    room_marker.id = 0;
    room_marker.type = visualization_msgs::Marker::CUBE_LIST;
    room_marker.color.r = 1;
    // room_marker.color.g = 0.07;
    // room_marker.color.b = 0.0;
    // room_marker.color.a = 1;

    for(const auto& room : room_vec.rooms) {
      geometry_msgs::Point point;
      point.x = room.room_center.x;
      point.y = room.room_center.y;
      point.z = 7.0;
      room_marker.points.push_back(point);

      std_msgs::ColorRGBA point_color;
      if(room.x_planes.size() == 2 && room.y_planes.size() == 2) {
        point_color.r = 1;
        point_color.a = 1;
      } else {
        point_color.g = 1;
        point_color.a = 1;
      }
      room_marker.colors.push_back(point_color);
    }

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(room_marker);
    room_centers_pub.publish(markers);
  }

private:
  ros::Subscriber skeleton_graph_sub;
  ros::Subscriber map_planes_sub;
  ros::Publisher cluster_cloud_pub;
  ros::Publisher cluster_clouds_pub;
  ros::Publisher room_data_pub;
  ros::Publisher room_centers_pub;
  ros::Publisher refined_skeleton_graph_pub;

  /* private variables */
private:
  ros::Timer room_detection_timer;
  std::mutex map_plane_mutex;

  // skeleton graph queue
  std::deque<visualization_msgs::MarkerArray::Ptr> skeleton_graph_queue;
  std::vector<s_graphs::PlaneData> x_vert_plane_vec, y_vert_plane_vec;
  std::deque<std::vector<s_graphs::PlaneData>> x_vert_plane_queue, y_vert_plane_queue;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  std::unique_ptr<RoomAnalyzer> room_analyzer;
  std::shared_ptr<PlaneUtils> plane_utils;
};

}  // namespace s_graphs

PLUGINLIB_EXPORT_CLASS(s_graphs::RoomSegmentationNodelet, nodelet::Nodelet)
