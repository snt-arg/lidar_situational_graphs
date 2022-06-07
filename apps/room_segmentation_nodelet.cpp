#include <iostream>
#include <string>
#include <cmath>
#include <math.h>

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
    map_planes_sub      = nh.subscribe("/s_graphs/map_planes",1,&RoomSegmentationNodelet::map_planes_callback, this);

    cluster_cloud_pub_   = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_cloud",1,true);
    cluster_clouds_pub_  = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_clouds",1,true);
    room_data_pub_       = nh.advertise<s_graphs::RoomsData>("/room_segmentation/room_data", 1, true);
    room_centers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/room_segmentation/room_centers", 1, true);
  }

  /**
   * @brief get the vertical planes in map frame 
   * @param map_planes_msg
   */
  void map_planes_callback(const s_graphs::PlanesData::Ptr& map_planes_msg) {
    x_vert_plane_vec = map_planes_msg->x_planes;
    y_vert_plane_vec = map_planes_msg->y_planes;
  }

  /**
   * 
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
      std::cout << "cluster " << i << " has x min and x max: " << p1.x << ", " << p2.x << std::endl;
      std::cout << "cluster " << i << " has y min and y max: " << p1.y << ", " << p2.y << std::endl;
      geometry_msgs::Point room_length = get_room_length(p1,p2);
      //std::cout << "length of the cluster in x : " << room_length.x << std::endl;    
      //std::cout << "length of the cluster in y : " << room_length.y << std::endl;    
      float room_diff_thres = 2.0; 
      bool found_all_planes = false;
      s_graphs::PlaneData x_plane1, x_plane2, y_plane1, y_plane2;
      found_all_planes = get_room_planes(p1,p2, x_plane1, x_plane2, y_plane1, y_plane2);

      if(found_all_planes) {
        geometry_msgs::Point room_center = get_room_center(p1, p2, x_plane1, x_plane2, y_plane1, y_plane2);
        //std::cout << "temp room center is: " << room_center.x << " , " << room_center.y << std::endl; 

        s_graphs::RoomData room_candidate;
        room_candidate.id = i;
        room_candidate.room_length = room_length;
        room_candidate.room_center = room_center;
        room_candidates_vec.push_back(room_candidate);
        i++;
      }else {
        i++;
        continue;
      }

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

  geometry_msgs::Point get_room_center(pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData x_plane1, s_graphs::PlaneData x_plane2, s_graphs::PlaneData y_plane1, s_graphs::PlaneData y_plane2) {
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

  bool get_room_planes(pcl::PointXY p_min, pcl::PointXY p_max, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2) {
    float room_dist_thres = 1.0;
    float min_dist_x1 =100; float min_dist_x2 =100;
    bool found_x1_plane = false, found_x2_plane = false, found_y1_plane = false, found_y2_plane = false;

    for(const auto& x_plane : x_vert_plane_vec) {
      //std::cout << "xplane: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;

      float dist_x1 = -1*(x_plane.nx * p_min.x + x_plane.ny * p_min.y);
      float diff_dist_x1 = 100;
      if((dist_x1 * x_plane.d) > 0)
        diff_dist_x1 = fabs(dist_x1 - x_plane.d);

      //std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;
      if(diff_dist_x1 < min_dist_x1) {
        min_dist_x1 = diff_dist_x1;
        x_plane1 = x_plane;
      }

      float dist_x2 = -1*(x_plane.nx * p_max.x + x_plane.ny * p_max.y);
      float diff_dist_x2 = 100;
      if((dist_x2 * x_plane.d) > 0)
        diff_dist_x2 = fabs(dist_x2 - x_plane.d);
      //std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;

      if(diff_dist_x2 < min_dist_x2) {
        min_dist_x2 = diff_dist_x2;
        x_plane2 = x_plane;
      }
    }

    if(x_plane1.d - x_plane2.d == 0) {
      std::cout << "no xplane1 found " << std::endl;    
      std::cout << "no xplane2 found " << std::endl;
      found_x1_plane = false, found_x2_plane = false;    
    } else {
      if(min_dist_x1 < room_dist_thres) {
        std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
        found_x1_plane = true;
      }
      else  {
        std::cout << "no xplane1 found " << std::endl;  
        found_x1_plane = false;
      }
      if(min_dist_x2 < room_dist_thres) {
        std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;  
        found_x2_plane = true;
      }  
      else {
        std::cout << "no xplane2 found " << std::endl;   
        found_x2_plane = false;
      }
    }

    float min_dist_y1 =100; float min_dist_y2 =100;
    for(const auto& y_plane : y_vert_plane_vec) {
      //std::cout << "yplane: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;

      float dist_y1 = -1*(y_plane.nx * p_min.x + y_plane.ny * p_min.y);
      float diff_dist_y1 = 100;
      if((dist_y1 * y_plane.d) > 0)
        diff_dist_y1 = fabs(dist_y1 - y_plane.d);

      //std::cout << "diff dist y1: " << diff_dist_y1 << std::endl;
      if(diff_dist_y1 < min_dist_y1) {
        min_dist_y1 = diff_dist_y1;
        y_plane1 = y_plane;
      }

      float dist_y2 = -1*(y_plane.nx * p_max.x + y_plane.ny * p_max.y);
      float diff_dist_y2 = 100;
      if((dist_y2 * y_plane.d) > 0)
        diff_dist_y2 = fabs(dist_y2 - y_plane.d);
      //std::cout << "diff dist y2: " << diff_dist_y2 << std::endl;

      if(diff_dist_y2 < min_dist_y2) {
        min_dist_y2 = diff_dist_y2;
        y_plane2 = y_plane;
      }
    }

    if(y_plane1.d - y_plane2.d == 0) {
      std::cout << "no yplane1 found " << std::endl;    
      std::cout << "no yplane2 found " << std::endl;  
      found_y1_plane = false, found_y2_plane = false;
    } else {
      if(min_dist_y1 < room_dist_thres) {
        std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
        found_y1_plane = true;
      }
      else {
        std::cout << "no yplane1 found " << std::endl;    
        found_y1_plane = false;
      }
      if(min_dist_y2 < room_dist_thres) {
        std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;  
        found_y2_plane = true;
      } 
      else {
        std::cout << "no yplane2 found " << std::endl;   
        found_y2_plane = false;
      }
    }
    if(found_x1_plane && found_x2_plane && found_y1_plane && found_y2_plane)
      return true;
    else 
      return false;
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
  ros::Subscriber map_planes_sub;
  ros::Publisher  cluster_cloud_pub_;
  ros::Publisher  cluster_clouds_pub_;
  ros::Publisher  room_data_pub_;
  ros::Publisher  room_centers_pub_;

  /* private variables */
private:
  //skeleton graph queue
  std::deque<visualization_msgs::MarkerArray::Ptr> skeleton_graph_queue; 
  std::vector<s_graphs::PlaneData> x_vert_plane_vec, y_vert_plane_vec;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
};

}  // namespace room segmentation

PLUGINLIB_EXPORT_CLASS(s_graphs::RoomSegmentationNodelet, nodelet::Nodelet)
