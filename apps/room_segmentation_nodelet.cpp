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
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ml/kmeans.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

bool customRegionGrowing(const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float) {
  double point_angle_thres = 60;
  // std::cout << "Point a: " << point_a.x << ", " << point_a.y << std::endl;
  // std::cout << "Point b: " << point_b.x << ", " << point_b.y << std::endl;
  double diff_y = fabs(point_b.y - point_a.y);
  double diff_x = fabs(point_b.x - point_a.x);
  // std::cout << "diff_y: " << diff_y << std::endl;
  // std::cout << "diff_x: " << diff_x << std::endl;

  double angle = atan2(diff_x, diff_y) * 180 / M_PI;
  if(angle > point_angle_thres) {
    std::cout << "angle: " << angle << std::endl;
    return false;
  } else
    return true;
}

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
  void initialize_params() {}

  void init_ros() {
    skeleton_graph_sub = nh.subscribe("/voxblox_skeletonizer/sparse_graph", 1, &RoomSegmentationNodelet::skeleton_graph_callback, this);
    map_planes_sub = nh.subscribe("/s_graphs/map_planes", 100, &RoomSegmentationNodelet::map_planes_callback, this);

    cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_cloud", 1, true);
    hull_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/hull_cloud", 1, true);
    cluster_clouds_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_clouds", 1, true);
    room_data_pub_ = nh.advertise<s_graphs::RoomsData>("/room_segmentation/room_data", 1, true);
    room_centers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/room_centers", 1, true);
    room_diagonal_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/room_diagonal", 1, true);

    room_detection_timer = nh.createWallTimer(ros::WallDuration(3.0), &RoomSegmentationNodelet::room_detection_callback, this);
  }

  template<typename T>
  bool contains(std::vector<T> vec, const T& elem) {
    bool result = false;
    if(find(vec.begin(), vec.end(), elem) != vec.end()) {
      result = true;
    }
    return result;
  }

  void room_detection_callback(const ros::WallTimerEvent& event) {
    std::vector<s_graphs::PlaneData> current_x_vert_planes, current_y_vert_planes;
    flush_map_planes(current_x_vert_planes, current_y_vert_planes);

    if(current_x_vert_planes.empty() && current_y_vert_planes.empty()) return;

    auto t1 = ros::WallTime::now();
    extract_rooms(current_x_vert_planes, current_y_vert_planes);
    auto t2 = ros::WallTime::now();
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
    flush_skeleton_graph_queue(skeleton_graph_msg);
  }

  /**
   * @brief flush the skeleton graph queue
   *
   */
  bool flush_skeleton_graph_queue(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
    cloud_clusters_.clear();
    connected_subgraphs_.clear();

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters;
    int subgraph_id = 0;
    std::vector<std::pair<int, int>> connected_subgraph_map;
    for(const auto& single_graph : skeleton_graph_msg->markers) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::string vertex_string = "connected_vertices_";
      size_t found = single_graph.ns.find(vertex_string);
      if(found != std::string::npos) {
        float r = rand() % 256;
        float g = rand() % 256;
        float b = rand() % 256;
        for(size_t i = 0; i < single_graph.points.size(); ++i) {
          pcl::PointXYZRGB pcl_point;
          pcl_point.x = single_graph.points[i].x;
          pcl_point.y = single_graph.points[i].y;
          pcl_point.z = 7.0;
          pcl_point.r = r;
          pcl_point.g = g;
          pcl_point.b = b;
          tmp_cloud_cluster->points.push_back(pcl_point);
        }
        // insert subgraph id in the seq
        tmp_cloud_cluster->header.seq = subgraph_id;
        curr_cloud_clusters.push_back(tmp_cloud_cluster);
        subgraph_id++;
      }

      // get the connected subsgraphs
      std::string connected_subgraph_string = "subgraph_edges_";
      size_t found_connection = single_graph.ns.find(connected_subgraph_string);
      if(found_connection != std::string::npos) {
        // the position here encodes the two subgraph ids.
        int subgraph_1_id = single_graph.id >> 8;
        int subgraph_2_id = single_graph.id & (2 * 2 * 2 * 2 - 1);

        bool pair_exists = false;
        for(const auto& connected_graph_ids : connected_subgraph_map) {
          if(connected_graph_ids.first == subgraph_1_id && connected_graph_ids.second == subgraph_2_id) {
            pair_exists = true;
            continue;
          } else if(connected_graph_ids.first == subgraph_2_id && connected_graph_ids.second == subgraph_1_id) {
            pair_exists = true;
            continue;
          }
        }

        if(pair_exists) continue;

        // std::cout << "subgraph_1_id:" << subgraph_1_id << std::endl;
        // std::cout << "subgraph_2_id:" << subgraph_2_id << std::endl;

        std::pair<int, int> connected_subgraph;
        connected_subgraph = std::make_pair(subgraph_1_id, subgraph_2_id);
        connected_subgraph_map.push_back(connected_subgraph);
      }
    }

    skeleton_graph_mutex.lock();
    cloud_clusters_ = curr_cloud_clusters;
    connected_subgraphs_ = connected_subgraph_map;
    skeleton_graph_mutex.unlock();

    return true;
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud
   *
   */
  void extract_rooms(std::vector<s_graphs::PlaneData> current_x_vert_planes, std::vector<s_graphs::PlaneData> current_y_vert_planes) {
    int room_cluster_counter = 0;
    std::vector<s_graphs::RoomData> room_candidates_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visualizer(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_visualizer(new pcl::PointCloud<pcl::PointXYZRGB>);
    diag_line_viz_vec_.clear();

    skeleton_graph_mutex.lock();
    std::vector<std::pair<int, int>> connected_subgraph_map = connected_subgraphs_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters = cloud_clusters_;
    skeleton_graph_mutex.unlock();

    for(const auto& cloud_cluster : curr_cloud_clusters) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);

      if(cloud_cluster->points.size() < 10) continue;

      float hull_area;
      get_convex_hull(cloud_cluster, cloud_hull, hull_area);
      if(hull_area < 1.5) {
        std::cout << "subgraph area too small to be a room " << std::endl;
        continue;
      }

      perform_room_segmentation(current_x_vert_planes, current_y_vert_planes, room_cluster_counter, cloud_cluster, cloud_hull, room_candidates_vec, connected_subgraph_map);
      for(int i = 0; i < cloud_cluster->points.size(); ++i) {
        cloud_visualizer->points.push_back(cloud_cluster->points[i]);
      }
    }

    s_graphs::RoomsData room_candidates_msg;
    room_candidates_msg.header.stamp = ros::Time::now();
    room_candidates_msg.rooms = room_candidates_vec;
    room_data_pub_.publish(room_candidates_msg);
    viz_room_centers(room_candidates_msg);

    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_visualizer, cloud_cluster_msg);
    cloud_cluster_msg.header.stamp = ros::Time::now();
    cloud_cluster_msg.header.frame_id = "map";
    cluster_cloud_pub_.publish(cloud_cluster_msg);
  }

  void perform_room_segmentation(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, int& room_cluster_counter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, std::vector<s_graphs::RoomData>& room_candidates_vec, std::vector<std::pair<int, int>> connected_subgraph_map) {
    pcl::PointXY p1;
    pcl::PointXY p2;
    bool centroid_inside = get_centroid_location(cloud_cluster, p1, p2);

    // we have encountered an irregular shape here
    // TODO:HB perform clustering of the cloud_cluster into subspaces
    if(!centroid_inside) {
      std::cout << "returning as its a weird shape and we dont know how to handle it right now " << std::endl;
      return;
    }

    // cluster_endpoints(cloud_cluster, p1, p2);
    std::cout << "cluster " << room_cluster_counter << " has x min and x max: " << p1.x << ", " << p2.x << std::endl;
    std::cout << "cluster " << room_cluster_counter << " has y min and y max: " << p1.y << ", " << p2.y << std::endl;
    geometry_msgs::Point room_length = get_room_length(p1, p2);
    // std::cout << "length of the cluster in x : " << room_length.x << std::endl;
    // std::cout << "length of the cluster in y : " << room_length.y << std::endl;

    // TODO:HB check room width here
    if(room_length.x < 0.5 || room_length.y < 0.5) {
      room_cluster_counter++;
    } else {
      // check how many planes are extracted
      // if four planes are found its a bounded room
      // if 2 parallel planes are found it an ifnite corridor
      bool found_all_planes = false;
      bool found_x1_plane = false;
      bool found_x2_plane = false;
      bool found_y1_plane = false;
      bool found_y2_plane = false;
      s_graphs::PlaneData x_plane1, x_plane2, y_plane1, y_plane2;
      get_room_planes(current_x_vert_planes, current_y_vert_planes, p1, p2, cloud_hull, x_plane1, x_plane2, y_plane1, y_plane2, found_x1_plane, found_x2_plane, found_y1_plane, found_y2_plane);
      // clear plane points which are not required now
      x_plane1.plane_points.clear();
      x_plane2.plane_points.clear();
      y_plane1.plane_points.clear();
      y_plane2.plane_points.clear();

      // if found all four planes its a room
      if(found_x1_plane && found_x2_plane && found_y1_plane && found_y2_plane) {
        correct_plane_d(plane_class::X_VERT_PLANE, x_plane1);
        correct_plane_d(plane_class::X_VERT_PLANE, x_plane2);
        correct_plane_d(plane_class::Y_VERT_PLANE, y_plane1);
        correct_plane_d(plane_class::Y_VERT_PLANE, y_plane2);

        // first check the width of the rooms
        bool is_x_plane_wide = room_width(x_plane1, x_plane2);
        bool is_y_plane_wide = room_width(y_plane1, y_plane2);

        if(!is_x_plane_wide || !is_y_plane_wide) {
          std::cout << "returning as the room is not sufficiently wide" << std::endl;
          return;
        }

        std::vector<int> neighbour_ids;
        for(const auto& connected_ids : connected_subgraph_map) {
          if(connected_ids.first == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.second);
          else if(connected_ids.second == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.first);
        }
        geometry_msgs::Point room_center = get_room_center(p1, p2, x_plane1, x_plane2, y_plane1, y_plane2);
        s_graphs::RoomData room_candidate;
        room_candidate.id = cloud_cluster->header.seq;
        room_candidate.neighbour_ids = neighbour_ids;
        room_candidate.room_length = room_length;
        room_candidate.room_center = room_center;
        room_candidate.x_planes.push_back(x_plane1);
        room_candidate.x_planes.push_back(x_plane2);
        room_candidate.y_planes.push_back(y_plane1);
        room_candidate.y_planes.push_back(y_plane2);
        room_candidates_vec.push_back(room_candidate);
        room_cluster_counter++;
      }
      // if found only two x planes are found at x corridor
      else if(found_x1_plane && found_x2_plane && (!found_y1_plane || !found_y2_plane)) {
        correct_plane_d(plane_class::X_VERT_PLANE, x_plane1);
        correct_plane_d(plane_class::X_VERT_PLANE, x_plane2);

        bool is_x_plane_wide = room_width(x_plane1, x_plane2);
        if(!is_x_plane_wide) {
          std::cout << "returning as the room is not sufficiently wide" << std::endl;
          return;
        }

        std::vector<int> neighbour_ids;
        for(const auto& connected_ids : connected_subgraph_map) {
          if(connected_ids.first == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.second);
          else if(connected_ids.second == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.first);
        }

        geometry_msgs::Point room_center = get_corridor_center(plane_class::X_VERT_PLANE, p1, p2, x_plane1, x_plane2);
        s_graphs::RoomData room_candidate;
        room_candidate.id = cloud_cluster->header.seq;
        room_candidate.neighbour_ids = neighbour_ids;
        room_candidate.room_length = room_length;
        room_candidate.room_center = room_center;
        room_candidate.x_planes.push_back(x_plane1);
        room_candidate.x_planes.push_back(x_plane2);
        room_candidates_vec.push_back(room_candidate);
        room_cluster_counter++;
      }
      // if found only two y planes are found at y corridor
      else if(found_y1_plane && found_y2_plane && (!found_x1_plane || !found_x2_plane)) {
        correct_plane_d(plane_class::Y_VERT_PLANE, y_plane1);
        correct_plane_d(plane_class::Y_VERT_PLANE, y_plane2);

        bool is_y_plane_wide = room_width(y_plane1, y_plane2);
        if(!is_y_plane_wide) {
          std::cout << "returning as the room is not sufficiently wide" << std::endl;
          return;
        }

        std::vector<int> neighbour_ids;
        for(const auto& connected_ids : connected_subgraph_map) {
          if(connected_ids.first == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.second);
          else if(connected_ids.second == cloud_cluster->header.seq)
            neighbour_ids.push_back(connected_ids.first);
        }

        geometry_msgs::Point room_center = get_corridor_center(plane_class::Y_VERT_PLANE, p1, p2, y_plane1, y_plane2);
        s_graphs::RoomData room_candidate;
        room_candidate.id = cloud_cluster->header.seq;
        room_candidate.neighbour_ids = neighbour_ids;
        room_candidate.room_length = room_length;
        room_candidate.room_center = room_center;
        room_candidate.y_planes.push_back(y_plane1);
        room_candidate.y_planes.push_back(y_plane2);
        room_candidates_vec.push_back(room_candidate);
      } else {
        room_cluster_counter++;
      }
    }
  }

  bool get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, pcl::PointXY& p1, pcl::PointXY& p2) {
    pcl::PointXYZRGB min, max;
    pcl::getMinMax3D(*skeleton_cloud, min, max);
    p1.x = min.x;
    p1.y = min.y;
    p2.x = max.x;
    p2.y = max.y;

    pcl::PointXYZRGB centroid = get_centroid(min, max);

    // get the dist of centroid wrt to all the points in the cluster
    // centroids lying outside will have higher distance to the points in the cluster
    float min_dist = 100;
    for(size_t i = 0; i < skeleton_cloud->points.size(); ++i) {
      float dist = sqrt(pow(centroid.x - skeleton_cloud->points[i].x, 2) + pow(centroid.y - skeleton_cloud->points[i].y, 2));

      if(dist < min_dist) {
        min_dist = dist;
      }
    }

    if(min_dist > 0.5) {
      std::cout << "centroid outside the cluster! Do something " << std::endl;
      return false;
    }

    return true;
  }

  void get_convex_hull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& area) {
    // Create a convex hull representation of the projected inliers
    pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;

    std::vector<pcl::Vertices> polygons;
    convex_hull.setInputCloud(skeleton_cloud);
    // chull.setAlpha (0.1);
    convex_hull.setDimension(2);
    convex_hull.setComputeAreaVolume(true);
    convex_hull.reconstruct(*cloud_hull, polygons);
    // std::cout << "polygons before: " << polygons[0].vertices.size() << std::endl;
    area = convex_hull.getTotalArea();

    return;
  }

  pcl::PointXYZRGB get_centroid(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
    pcl::PointXYZRGB center;
    if(fabs(p1.x) > fabs(p2.x)) {
      float size = p1.x - p2.x;
      center.x = (size / 2) + p2.x;
    } else {
      float size = p2.x - p1.x;
      center.x = (size / 2) + p1.x;
    }

    if(fabs(p1.y) > fabs(p2.y)) {
      float size = p1.y - p2.y;
      center.y = (size / 2) + p2.y;
    } else {
      float size = p2.y - p1.y;
      center.y = (size / 2) + p1.y;
    }

    center.z = p1.z;
    return center;
  }

  bool get_diagonal_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const pcl::PointXY min, const pcl::PointXY max, pcl::PointXY& top_right, pcl::PointXY& bottom_right, pcl::PointXY& top_left, pcl::PointXY& bottom_left) {
    // get the set of diagonal points
    pcl::PointXY bottom_right_img, top_right_img, bottom_left_img, top_left_img;
    bottom_right_img.x = min.x;
    bottom_right_img.y = min.y;
    top_left_img.x = max.x;
    top_left_img.y = max.y;

    // this is imaginary point and based on this we will find the closest point in our hull
    bottom_left_img.x = min.x;
    bottom_left_img.y = max.y;
    top_right_img.x = max.x;
    top_right_img.y = min.y;

    // find the neareast neighbour to our imaginary points in our hull
    float min_dist_top_right = 100;
    float min_dist_bottom_right = 100;
    float min_dist_top_left = 100;
    float min_dist_bottom_left = 100;

    int top_right_index = -1;
    int bottom_right_index = -1;
    int top_left_index = -1;
    int bottom_left_index = -1;

    for(int i = 0; i < cloud_hull->points.size(); ++i) {
      float dist_top_right = sqrt(pow(cloud_hull->points[i].x - top_right_img.x, 2) + pow(cloud_hull->points[i].y - top_right_img.y, 2));
      if(dist_top_right < min_dist_top_right) {
        min_dist_top_right = dist_top_right;
        top_right_index = i;
      }

      float dist_bottom_right = sqrt(pow(cloud_hull->points[i].x - bottom_right_img.x, 2) + pow(cloud_hull->points[i].y - bottom_right_img.y, 2));
      if(dist_bottom_right < min_dist_bottom_right) {
        min_dist_bottom_right = dist_bottom_right;
        bottom_right_index = i;
      }

      float dist_top_left = sqrt(pow(cloud_hull->points[i].x - top_left_img.x, 2) + pow(cloud_hull->points[i].y - top_left_img.y, 2));
      if(dist_top_left < min_dist_top_left) {
        min_dist_top_left = dist_top_left;
        top_left_index = i;
      }

      float dist_bottom_left = sqrt(pow(cloud_hull->points[i].x - bottom_left_img.x, 2) + pow(cloud_hull->points[i].y - bottom_left_img.y, 2));
      if(dist_bottom_left < min_dist_bottom_left) {
        min_dist_bottom_left = dist_bottom_left;
        bottom_left_index = i;
      }
    }

    if(top_right_index != -1 && bottom_right_index != -1 && top_left_index != -1 && bottom_left_index != -1) {
      if(min_dist_top_right < 1.0 && min_dist_bottom_left < 1.0) {
        top_right.x = cloud_hull->points[top_right_index].x;
        top_right.y = cloud_hull->points[top_right_index].y;

        bottom_right.x = cloud_hull->points[bottom_right_index].x;
        bottom_right.y = cloud_hull->points[bottom_right_index].y;

        top_left.x = cloud_hull->points[top_left_index].x;
        top_left.y = cloud_hull->points[top_left_index].y;

        bottom_left.x = cloud_hull->points[bottom_left_index].x;
        bottom_left.y = cloud_hull->points[bottom_left_index].y;

        return true;
      }
    }
    return false;
  }

  geometry_msgs::Point get_room_length(pcl::PointXY p1, pcl::PointXY p2) {
    geometry_msgs::Point length;
    if(fabs(p1.x) > fabs(p2.x)) {
      length.x = fabs(p1.x - p2.x);
    } else {
      length.x = fabs(p2.x - p1.x);
    }

    if(fabs(p1.y) > fabs(p2.y)) {
      length.y = fabs(p1.y - p2.y);
    } else {
      length.y = fabs(p2.y - p1.y);
    }

    return length;
  }

  geometry_msgs::Point get_room_center(pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData x_plane1, s_graphs::PlaneData x_plane2, s_graphs::PlaneData y_plane1, s_graphs::PlaneData y_plane2) {
    geometry_msgs::Point center;

    if(fabs(x_plane1.d) > fabs(x_plane2.d)) {
      double size = x_plane1.d - x_plane2.d;
      center.x = (((size) / 2) + x_plane2.d);
    } else {
      double size = x_plane2.d - x_plane1.d;
      center.x = (((size) / 2) + x_plane1.d);
    }

    if(fabs(y_plane1.d) > fabs(y_plane2.d)) {
      double size = y_plane1.d - y_plane2.d;
      center.y = (((size) / 2) + y_plane2.d);
    } else {
      double size = y_plane2.d - y_plane1.d;
      center.y = (((size) / 2) + y_plane1.d);
    }

    return center;
  }

  geometry_msgs::Point get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2) {
    geometry_msgs::Point center;

    if(plane_type == plane_class::X_VERT_PLANE) {
      if(fabs(plane1.d) > fabs(plane2.d)) {
        double size = plane1.d - plane2.d;
        center.x = (((size) / 2) + plane2.d);
      } else {
        double size = plane2.d - plane1.d;
        center.x = (((size) / 2) + plane1.d);
      }

      if(fabs(p1.y) > fabs(p2.y)) {
        float size = p1.y - p2.y;
        center.y = (size / 2) + p2.y;
      } else {
        float size = p2.y - p1.y;
        center.y = (size / 2) + p1.y;
      }
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      if(fabs(plane1.d) > fabs(plane2.d)) {
        double size = plane1.d - plane2.d;
        center.y = (((size) / 2) + plane2.d);
      } else {
        double size = plane2.d - plane1.d;
        center.y = (((size) / 2) + plane1.d);
      }

      if(fabs(p1.x) > fabs(p2.x)) {
        float size = p1.x - p2.x;
        center.x = (size / 2) + p2.x;
      } else {
        float size = p2.x - p1.x;
        center.x = (size / 2) + p1.x;
      }
    }

    return center;
  }

  void correct_plane_d(int plane_type, s_graphs::PlaneData& plane) {
    if(plane_type == plane_class::X_VERT_PLANE) {
      plane.d = -1 * plane.d;
      double p_norm = plane.nx / fabs(plane.nx);
      plane.d = p_norm * plane.d;
    }

    if(plane_type == plane_class::Y_VERT_PLANE) {
      plane.d = -1 * plane.d;
      double p_norm = plane.ny / fabs(plane.ny);
      plane.d = p_norm * plane.d;
    }
  }

  bool room_width(s_graphs::PlaneData& plane1, s_graphs::PlaneData& plane2) {
    float size = 0;
    float room_width_threshold = 1.0;
    if(fabs(plane1.d) > fabs(plane2.d))
      size = fabs(plane1.d - plane2.d);
    else if(fabs(plane2.d) > fabs(plane1.d))
      size = fabs(plane2.d - plane1.d);

    if(size > room_width_threshold) return true;

    return false;
  }

  void get_room_planes(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, pcl::PointXY p_min, pcl::PointXY p_max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2, bool& found_x1_plane, bool& found_x2_plane, bool& found_y1_plane, bool& found_y2_plane) {
    float room_dist_thres = 1.2;
    float plane_point_dist_thres = 1.2;
    float min_dist_x1 = 100;
    float min_dist_x2 = 100;
    std::vector<float> min_x1_plane_points_dist;
    min_x1_plane_points_dist.resize(2);
    min_x1_plane_points_dist[0] = 100;
    min_x1_plane_points_dist[1] = 100;
    std::vector<float> min_x2_plane_points_dist;
    min_x2_plane_points_dist.resize(2);
    min_x2_plane_points_dist[0] = 100;
    min_x2_plane_points_dist[1] = 100;

    pcl::PointXY top_right, bottom_right, top_left, bottom_left;
    if(!get_diagonal_points(cloud_hull, p_min, p_max, top_right, bottom_right, top_left, bottom_left)) {
      std::cout << "didnt not get diagonal points" << std::endl;
      return;
    }

    for(const auto& x_plane : current_x_vert_planes) {
      if(x_plane.nx < 0) {
        continue;
      }

      std::cout << "xplane1: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
      float dist_x1 = -1 * (x_plane.nx * p_min.x + x_plane.ny * p_min.y);
      float diff_dist_x1 = 100;
      diff_dist_x1 = sqrt((dist_x1 - x_plane.d) * (dist_x1 - x_plane.d));

      // std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;

      std::vector<float> diff_x_plane_points_dist = find_plane_points(bottom_left, bottom_right, x_plane);
      if(diff_x_plane_points_dist[0] < min_x1_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x1_plane_points_dist[1]) {
        min_dist_x1 = diff_dist_x1;
        min_x1_plane_points_dist[0] = diff_x_plane_points_dist[0];
        min_x1_plane_points_dist[1] = diff_x_plane_points_dist[1];

        x_plane1 = x_plane;
      }
    }

    for(const auto& x_plane : current_x_vert_planes) {
      if(x_plane.nx > 0) {
        continue;
      }

      std::cout << "xplane2: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
      float dist_x2 = -1 * (x_plane.nx * p_max.x + x_plane.ny * p_max.y);
      float diff_dist_x2 = 100;
      diff_dist_x2 = sqrt((dist_x2 - x_plane.d) * (dist_x2 - x_plane.d));

      // std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;
      std::vector<float> diff_x_plane_points_dist = find_plane_points(top_right, top_left, x_plane);
      if(diff_x_plane_points_dist[0] < min_x2_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x2_plane_points_dist[1]) {
        min_dist_x2 = diff_dist_x2;
        min_x2_plane_points_dist[0] = diff_x_plane_points_dist[0];
        min_x2_plane_points_dist[1] = diff_x_plane_points_dist[1];

        x_plane2 = x_plane;
      }
    }

    std::cout << "selected xplane1 : " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
    std::cout << "min_dist_x1 : " << min_dist_x1 << std::endl;
    std::cout << "min_x1_plane_points_dist[0] : " << min_x1_plane_points_dist[0] << std::endl;
    std::cout << "min_x1_plane_points_dist[1] : " << min_x1_plane_points_dist[1] << std::endl;

    std::cout << "selected xplane2 : " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
    std::cout << "min_dist_x2 : " << min_dist_x2 << std::endl;
    std::cout << "min_x2_plane_points_dist[0] : " << min_x2_plane_points_dist[0] << std::endl;
    std::cout << "min_x2_plane_points_dist[1] : " << min_x2_plane_points_dist[1] << std::endl;

    if(x_plane1.nx * x_plane2.nx > 0) {
      // std::cout << "no xplane1 found " << std::endl;
      // std::cout << "no xplane2 found " << std::endl;
      found_x1_plane = false, found_x2_plane = false;
    } else {
      if(min_dist_x1 < room_dist_thres && min_x1_plane_points_dist[0] < plane_point_dist_thres && min_x1_plane_points_dist[1] < plane_point_dist_thres) {
        std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
        found_x1_plane = true;
      } else {
        std::cout << "no xplane1 found " << std::endl;
        found_x1_plane = false;
      }
      if(min_dist_x2 < room_dist_thres && min_x2_plane_points_dist[0] < plane_point_dist_thres && min_x2_plane_points_dist[1] < plane_point_dist_thres) {
        std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
        found_x2_plane = true;
      } else {
        std::cout << "no xplane2 found " << std::endl;
        found_x2_plane = false;
      }
    }

    float min_dist_y1 = 100;
    float min_dist_y2 = 100;
    std::vector<float> min_y1_plane_points_dist;
    min_y1_plane_points_dist.resize(2);
    min_y1_plane_points_dist[0] = 100;
    min_y1_plane_points_dist[1] = 100;
    std::vector<float> min_y2_plane_points_dist;
    min_y2_plane_points_dist.resize(2);
    min_y2_plane_points_dist[0] = 100;
    min_y2_plane_points_dist[1] = 100;

    for(const auto& y_plane : current_y_vert_planes) {
      if(y_plane.ny < 0) {
        continue;
      }

      std::cout << "y_plane1: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
      float dist_y1 = -1 * (y_plane.nx * p_min.x + y_plane.ny * p_min.y);
      float diff_dist_y1 = 100;
      diff_dist_y1 = sqrt((dist_y1 - y_plane.d) * (dist_y1 - y_plane.d));

      // std::cout << "diff dist y1: " << diff_dist_y1 << std::endl;
      std::vector<float> diff_plane_points_dist = find_plane_points(top_right, bottom_right, y_plane);
      if(diff_plane_points_dist[0] < min_y1_plane_points_dist[0] && diff_plane_points_dist[1] < min_y1_plane_points_dist[1]) {
        min_dist_y1 = diff_dist_y1;
        min_y1_plane_points_dist[0] = diff_plane_points_dist[0];
        min_y1_plane_points_dist[1] = diff_plane_points_dist[1];
        y_plane1 = y_plane;
      }
    }

    for(const auto& y_plane : current_y_vert_planes) {
      if(y_plane.ny > 0) {
        continue;
      }

      std::cout << "y_plane2: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
      float dist_y2 = -1 * (y_plane.nx * p_max.x + y_plane.ny * p_max.y);
      float diff_dist_y2 = 100;
      diff_dist_y2 = sqrt((dist_y2 - y_plane.d) * (dist_y2 - y_plane.d));

      // std::cout << "diff dist y2: " << diff_dist_y2 << std::endl;
      std::vector<float> diff_plane_points_dist = find_plane_points(bottom_left, top_left, y_plane);
      if(diff_plane_points_dist[0] < min_y2_plane_points_dist[0] && diff_plane_points_dist[1] < min_y2_plane_points_dist[1]) {
        min_dist_y2 = diff_dist_y2;
        min_y2_plane_points_dist[0] = diff_plane_points_dist[0];
        min_y2_plane_points_dist[1] = diff_plane_points_dist[1];
        y_plane2 = y_plane;
      }
    }

    std::cout << "selected yplane1 : " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
    std::cout << "min_dist_y1 : " << min_dist_y1 << std::endl;
    std::cout << "min_y1_plane_points_dist[0] : " << min_y1_plane_points_dist[0] << std::endl;
    std::cout << "min_y1_plane_points_dist[1] : " << min_y1_plane_points_dist[1] << std::endl;

    std::cout << "selected yplane2 : " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
    std::cout << "min_dist_y2 : " << min_dist_y2 << std::endl;
    std::cout << "min_y2_plane_points_dist[0] : " << min_y2_plane_points_dist[0] << std::endl;
    std::cout << "min_y2_plane_points_dist[1] : " << min_y2_plane_points_dist[1] << std::endl;

    if(y_plane1.ny * y_plane2.ny > 0) {
      // std::cout << "no yplane1 found " << std::endl;
      // std::cout << "no yplane2 found " << std::endl;
      found_y1_plane = false, found_y2_plane = false;
    } else {
      if(min_dist_y1 < room_dist_thres && min_y1_plane_points_dist[0] < plane_point_dist_thres && min_y1_plane_points_dist[1] < plane_point_dist_thres) {
        std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
        found_y1_plane = true;
      } else {
        std::cout << "no yplane1 found " << std::endl;
        found_y1_plane = false;
      }
      if(min_dist_y2 < room_dist_thres && min_y2_plane_points_dist[0] < plane_point_dist_thres && min_y2_plane_points_dist[1] < plane_point_dist_thres) {
        std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
        found_y2_plane = true;
      } else {
        std::cout << "no yplane2 found " << std::endl;
        found_y2_plane = false;
      }
    }
  }

  std::vector<float> find_plane_points(const pcl::PointXY& start_point, const pcl::PointXY& end_point, const s_graphs::PlaneData& plane) {
    float min_start_point_plane_dist = 100;
    float min_end_point_plane_dist = 100;
    std::vector<float> plane_point_distances;
    geometry_msgs::Vector3 closest_start_plane_point, closest_end_plane_point;
    for(const auto& plane_point : plane.plane_points) {
      float start_plane_point_dist = sqrt(pow(start_point.x - plane_point.x, 2) + pow(start_point.y - plane_point.y, 2));

      float end_plane_point_dist = sqrt(pow(end_point.x - plane_point.x, 2) + pow(end_point.y - plane_point.y, 2));

      if(start_plane_point_dist < min_start_point_plane_dist) {
        min_start_point_plane_dist = start_plane_point_dist;
        closest_start_plane_point = plane_point;
      }

      if(end_plane_point_dist < min_end_point_plane_dist) {
        min_end_point_plane_dist = end_plane_point_dist;
        closest_end_plane_point = plane_point;
      }
    }

    std::cout << "closest_start_plane_point " << closest_start_plane_point.x << " ; " << closest_start_plane_point.y << std::endl;
    std::cout << "closest_end_plane_point " << closest_end_plane_point.x << " ; " << closest_end_plane_point.y << std::endl;
    plane_point_distances.push_back(min_start_point_plane_dist);
    plane_point_distances.push_back(min_end_point_plane_dist);

    return plane_point_distances;
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
    room_centers_pub_.publish(markers);
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> axis_clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud) {
    pcl::PointXYZRGB rand_number = skeleton_cloud->points[rand() % skeleton_cloud->points.size()];
    x_cluster_point x_centroid(rand_number.x, rand_number.y);
    x_centroid.cluster_id = 0;
    y_cluster_point y_centroid(rand_number.x, rand_number.y);
    y_centroid.cluster_id = 1;
    std::vector<cluster_point> points_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;

    for(int i = 0; i < skeleton_cloud->points.size(); ++i) {
      cluster_point point(skeleton_cloud->points[i].x, skeleton_cloud->points[i].y);
      points_vec.push_back(point);
    }

    int num_epochs = 0;
    while(num_epochs < 500) {
      for(int i = 0; i < points_vec.size(); ++i) {
        double dist = x_centroid.distance(points_vec[i]);
        if(dist < points_vec[i].min_dist) {
          points_vec[i].min_dist = dist;
          points_vec[i].cluster_id = x_centroid.cluster_id;
        }
      }

      for(int i = 0; i < points_vec.size(); ++i) {
        double dist = y_centroid.distance(points_vec[i]);
        if(dist < points_vec[i].min_dist) {
          points_vec[i].min_dist = dist;
          points_vec[i].cluster_id = y_centroid.cluster_id;
        }
      }

      std::vector<int> num_points;
      std::vector<double> sum_x;
      std::vector<double> sum_y;
      for(int i = 0; i < 2; i++) {
        num_points.push_back(0);
        sum_x.push_back(0);
        sum_y.push_back(0);
      }

      for(int i = 0; i < points_vec.size(); ++i) {
        int cluster_id = points_vec[i].cluster_id;
        num_points[cluster_id] += 1;
        sum_x[cluster_id] += points_vec[i].x;
        sum_y[cluster_id] += points_vec[i].y;
        points_vec[i].min_dist = 1000;
      }

      x_centroid.x = sum_x[0] / num_points[0];
      x_centroid.y = sum_y[0] / num_points[0];

      y_centroid.x = sum_x[1] / num_points[1];
      y_centroid.y = sum_y[1] / num_points[1];

      num_epochs++;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster2(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i = 0; i < points_vec.size(); ++i) {
      int cluster_id = points_vec[i].cluster_id;
      if(cluster_id == 0) {
        pcl::PointXYZRGB point;
        point.x = points_vec[i].x;
        point.y = points_vec[i].y;
        point.z = 7;
        point.r = 255;
        point.g = 0;
        point.b = 0;

        cloud_cluster1->points.push_back(point);
      }
      if(cluster_id == 1) {
        pcl::PointXYZRGB point;
        point.x = points_vec[i].x;
        point.y = points_vec[i].y;
        point.z = 7;
        point.r = 0;
        point.g = 0;
        point.b = 255;

        cloud_cluster2->points.push_back(point);
      }
    }
    cloud_clusters.push_back(cloud_cluster1);
    cloud_clusters.push_back(cloud_cluster2);
    std::cout << "x centroid: " << x_centroid.x << " ; " << x_centroid.y << std::endl;
    std::cout << "y centroid: " << y_centroid.x << " ; " << y_centroid.y << std::endl;

    return cloud_clusters;
  }

  void k_means_clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster) {
    pcl::Kmeans k_means_clustering(static_cast<int>(cloud_cluster->points.size()), 3);
    k_means_clustering.setClusterSize(2);
    for(size_t i = 0; i < cloud_cluster->points.size(); i++) {
      std::vector<float> data(3);
      data[0] = cloud_cluster->points[i].x;
      data[1] = cloud_cluster->points[i].y;
      data[2] = cloud_cluster->points[i].z;
      k_means_clustering.addDataPoint(data);
    }

    k_means_clustering.kMeans();
    pcl::Kmeans::Centroids centroids = k_means_clustering.get_centroids();
    std::cout << "centroid count: " << centroids.size() << std::endl;
    for(int i = 0; i < centroids.size(); i++) {
      std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
      std::cout << "y: " << centroids[i][1] << " ,";
      std::cout << "z: " << centroids[i][2] << std::endl;
    }
  }

  void cluster_endpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointXY& p1, pcl::PointXY& p2) {
    float max_x = -100, max_y = -100;
    float min_x = std::numeric_limits<float>::max(), min_y = std::numeric_limits<float>::max();

    for(size_t i = 0; i < cloud_cluster->points.size(); ++i) {
      if(cloud_cluster->points[i].x > max_x) {
        max_x = cloud_cluster->points[i].x;
      }
      if(cloud_cluster->points[i].y > max_y) {
        max_y = cloud_cluster->points[i].y;
      }
      if(cloud_cluster->points[i].x < min_x) {
        min_x = cloud_cluster->points[i].x;
      }
      if(cloud_cluster->points[i].y < min_y) {
        min_y = cloud_cluster->points[i].y;
      }
    }

    p1.x = min_x;
    p1.y = min_y;
    p2.x = max_x;
    p2.y = max_y;
  }

private:
  ros::Subscriber skeleton_graph_sub;
  ros::Subscriber map_planes_sub;
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher hull_cloud_pub_;
  ros::Publisher cluster_clouds_pub_;
  ros::Publisher room_data_pub_;
  ros::Publisher room_centers_pub_;
  ros::Publisher room_diagonal_pub_;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters_;
  std::vector<std::pair<int, int>> connected_subgraphs_;

  struct cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    cluster_point() : x(0.0), y(0.0), cluster_id(-1), min_dist(1000) {}

    cluster_point(double x, double y) : x(x), y(y), cluster_id(-1), min_dist(1000) {}
  };

  struct x_cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    x_cluster_point() : x(0.0), y(0.0), cluster_id(-1), min_dist(1000) {}

    x_cluster_point(double x, double y) : x(x), y(y), cluster_id(-1), min_dist(1000) {}

    double distance(cluster_point p) {
      return (p.x - x) * (p.x - x);
    }
  };

  struct y_cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    y_cluster_point() : x(0.0), y(0.0), cluster_id(-1), min_dist(1000) {}

    y_cluster_point(double x, double y) : x(x), y(y), cluster_id(-1), min_dist(1000) {}

    double distance(cluster_point p) {
      return (p.y - y) * (p.y - y);
    }
  };

  /* private variables */
private:
  ros::WallTimer room_detection_timer;
  std::mutex map_plane_mutex;
  std::mutex skeleton_graph_mutex;

  // skeleton graph queue
  std::deque<visualization_msgs::MarkerArray::Ptr> skeleton_graph_queue;
  std::vector<s_graphs::PlaneData> x_vert_plane_vec, y_vert_plane_vec;
  std::deque<std::vector<s_graphs::PlaneData>> x_vert_plane_queue, y_vert_plane_queue;
  std::vector<std::vector<pcl::PointXYZ>> diag_line_viz_vec_;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  enum plane_class : uint8_t {
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
  };
};

}  // namespace s_graphs

PLUGINLIB_EXPORT_CLASS(s_graphs::RoomSegmentationNodelet, nodelet::Nodelet)
