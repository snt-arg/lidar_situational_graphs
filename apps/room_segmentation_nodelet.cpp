#include <iostream>
#include <string>
#include <cmath>
#include <math.h>

#include <s_graphs/PointClouds.h>
#include <s_graphs/RoomData.h>
#include <s_graphs/RoomsData.h>

#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>

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
  }

  void init_ros() {
    skeleton_graph_sub  = nh.subscribe("/voxblox_skeletonizer/sparse_graph", 1, &RoomSegmentationNodelet::skeleton_graph_callback, this);  
  
    cluster_cloud_pub_   = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_cloud",1,true);
    cluster_clouds_pub_  = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_clouds",1,true);
    room_data_pub_       = nh.advertise<s_graphs::RoomsData>("/room_segmentation/room_data", 1, true);
    room_centers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/room_centers", 1, true);
  }

/**
   * @brief get the points from the skeleton graph for clusterting and identifying room candidates 
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
    skeleton_graph_queue.push_back(skeleton_graph_msg);
    flush_skeleton_graph_queue();
  }

  /**
   * @brief flush the skeleton graph queue
   * 
   */
  bool flush_skeleton_graph_queue() {
    //std::lock_guard<std::mutex> lock(skeleton_graph_mutex);
     pcl::PointCloud<pcl::PointXYZ>::Ptr skeleton_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for(const auto& skeleton_graph : skeleton_graph_queue) {
      for(const auto& single_skeleton : skeleton_graph->markers) {
          if(single_skeleton.ns == "vertices") {
            //std::cout << "single_skeleton vertices has points: " << single_skeleton.points.size() << std::endl;
            for(const auto& point : single_skeleton.points) {
             pcl::PointXYZ pcl_point;
             pcl_point.x =  point.x;
             pcl_point.y =  point.y;
             pcl_point.z =  7.0;
             skeleton_cloud->points.push_back(pcl_point);
            }
          }
      }
    }
    skeleton_graph_queue.clear();
    
    std::cout << "skeletal cloud size: " << skeleton_cloud->points.size() << std::endl;  
    extract_skeletal_clusters(skeleton_cloud);

    return true;
  }

  /**
   * @brief extract clusters with its centers from the skeletal cloud 
   * 
   */
  void extract_skeletal_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr skeleton_cloud) {
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (skeleton_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.5); 
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (skeleton_cloud);
    ec.extract (cluster_indices);

    std::cout << "cloud indices size: " << cluster_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      float r = rand()%256 ; float g = rand()%256; float b = rand()%256;
      for (const auto& idx : it->indices) {
        pcl::PointXYZRGB pointrgb;
        pointrgb.x = skeleton_cloud->points[idx].x;
        pointrgb.y = skeleton_cloud->points[idx].y;
        pointrgb.z = skeleton_cloud->points[idx].z;
        pointrgb.r = r;
        pointrgb.g = g;
        pointrgb.b = b;
        cloud_cluster->points.push_back(pointrgb);
        tmp_cloud_cluster->points.push_back(pointrgb);
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;     
      cloud_clusters.push_back(tmp_cloud_cluster);
    }

    int i=0;
    std::vector<s_graphs::RoomData> room_candidates_vec;
    for(const auto& cloud_cluster : cloud_clusters) {
      pcl::PointXY p1; pcl::PointXY p2;
      cluster_endpoints(cloud_cluster, p1, p2);
      //std::cout << "cluster " << i << " has x min and x max: " << p1.x << ", " << p2.x << std::endl;
      //std::cout << "cluster " << i << " has y min and y max: " << p1.y << ", " << p2.y << std::endl;
      geometry_msgs::Point room_length = get_room_length(p1,p2);
      //std::cout << "length of the cluster in x : " << room_length.x << std::endl;    
      //std::cout << "length of the cluster in y : " << room_length.y << std::endl;    
      geometry_msgs::Point room_center = get_room_center(p1, p2);
      //std::cout << "temp room center is: " << room_center.x << " , " << room_center.y << std::endl; 

      s_graphs::RoomData room_candidate;
      room_candidate.id = i;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidates_vec.push_back(room_candidate);
      i++;
    }  

    s_graphs::RoomsData room_candidates_msg;
    room_candidates_msg.header.stamp = ros::Time::now();
    room_candidates_msg.rooms = room_candidates_vec;
    room_data_pub_.publish(room_candidates_msg);
    viz_room_centers(room_candidates_msg);

    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);
    cloud_cluster_msg.header.stamp = ros::Time::now();
    cloud_cluster_msg.header.frame_id = "map";
    cluster_cloud_pub_.publish(cloud_cluster_msg);
  }


  void cluster_endpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointXY& p1, pcl::PointXY& p2) {
    pcl::PointXYZRGB pmin, pmax;
    pcl::getMaxSegment(*cloud_cluster, pmin, pmax);
    p1.x = pmin.x; p1.y = pmin.y;       
    p2.x = pmax.x; p2.y = pmax.y;
    //float length = pcl::euclideanDistance(p1,p2);

  } 

  geometry_msgs::Point get_room_length(pcl::PointXY p1, pcl::PointXY p2) {
    geometry_msgs::Point length; 
    if(fabs(p1.x) > fabs(p2.x)) {
      length.x = fabs(p1.x - p2.x);
    }else {
      length.x = fabs(p2.x - p1.x);
    }
    
    if(fabs(p1.y) > fabs(p2.y)) {
      length.y = fabs(p1.y - p2.y);
    }else {
      length.y = fabs(p2.y - p1.y);
    }

    return length;
  }

  geometry_msgs::Point get_room_center(pcl::PointXY p1, pcl::PointXY p2) {
    geometry_msgs::Point center; 
    if(fabs(p1.x) > fabs(p2.x)) {
      float size = p1.x - p2.x;
      center.x = (size/2) + p2.x;
    }else {
      float size = p2.x - p1.x;
      center.x = (size/2) + p1.x;
    }
    
    if(fabs(p1.y) > fabs(p2.y)) {
      float size = p1.y - p2.y;
      center.y = (size/2) + p2.y;
    }else {
      float size = p2.y - p1.y;
      center.y = (size/2) + p1.y;
    }

    return center;
  }

  void viz_room_centers(s_graphs::RoomsData room_vec) {
    visualization_msgs::Marker room_marker;
    room_marker.pose.orientation.w = 1.0;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    //plane_marker.points.resize(vert_planes.size());    
    room_marker.header.frame_id = "map";
    room_marker.header.stamp = ros::Time::now();
    room_marker.ns = "rooms";
    room_marker.id = 0;
    room_marker.type = visualization_msgs::Marker::CUBE_LIST;
    room_marker.color.r = 1;
    room_marker.color.g = 0.07;
    room_marker.color.b = 0.0;
    room_marker.color.a = 1; 

    for(const auto& room : room_vec.rooms) {
       geometry_msgs::Point point;
       point.x = room.room_center.x;
       point.y = room.room_center.y;
       point.z = 7.0;
       room_marker.points.push_back(point);
    }

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(room_marker);
    room_centers_pub_.publish(markers);
  }

private:
  ros::Subscriber skeleton_graph_sub;
  ros::Publisher  cluster_cloud_pub_;
  ros::Publisher  cluster_clouds_pub_;
  ros::Publisher  room_data_pub_;
  ros::Publisher  room_centers_pub_;

  /* private variables */
private:
  //skeleton graph queue
  std::deque<visualization_msgs::MarkerArray::Ptr> skeleton_graph_queue; 

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
};

}  // namespace room segmentation

PLUGINLIB_EXPORT_CLASS(s_graphs::RoomSegmentationNodelet, nodelet::Nodelet)
