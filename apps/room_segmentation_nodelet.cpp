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

bool customRegionGrowing(const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float) {
  double point_angle_thres = 60;
  //std::cout << "Point a: " << point_a.x << ", " << point_a.y << std::endl;
  //std::cout << "Point b: " << point_b.x << ", " << point_b.y << std::endl;
  double diff_y = fabs(point_b.y - point_a.y);
  double diff_x = fabs(point_b.x - point_a.x);
  //std::cout << "diff_y: " << diff_y << std::endl;
  //std::cout << "diff_x: " << diff_x << std::endl;

  double angle = atan2(diff_x , diff_y) * 180 / M_PI;
  if(angle > point_angle_thres) {
    std::cout << "angle: " << angle << std::endl;  
    return false;
  } 
  else
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
    flush_skeleton_graph_queue(skeleton_graph_msg);
  }

  /**
   * @brief flush the skeleton graph queue
   * 
   */
  bool flush_skeleton_graph_queue(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
    //std::lock_guard<std::mutex> lock(skeleton_graph_mutex);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;


    std::vector<std_msgs::ColorRGBA> color_msg_vec;
      for(const auto& single_skeleton : skeleton_graph_msg->markers) {          
          if(single_skeleton.ns == "vertices") {
            for(int i = 0; i < single_skeleton.colors.size(); ++i) {
               if(color_msg_vec.empty()){ 
                 color_msg_vec.push_back(single_skeleton.colors[i]);
               } else {
                //check if color exists in the vector
                 bool color_exists = false;
                 for(int j = 0; j < color_msg_vec.size(); ++j){
                   if(color_msg_vec[j] == single_skeleton.colors[i]){
                     color_exists =true;
                   }
                 }
                 if(!color_exists)
                  color_msg_vec.push_back(single_skeleton.colors[i]);
               }
            }
            
           std::cout << "total number of subgraphs: " <<  color_msg_vec.size() << std::endl;
          
          int color_code = 100;
          for(const auto& color_msg : color_msg_vec) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            float r = color_code%256 ; float g = color_code%256; float b = color_code%256;
            for(const auto& single_skeleton : skeleton_graph_msg->markers) {
              if(single_skeleton.ns == "vertices") {
                for(size_t i=0; i<single_skeleton.points.size(); ++i) {
                  if(single_skeleton.colors[i] == color_msg) {
                    pcl::PointXYZRGB pcl_point;
                    pcl_point.x =  single_skeleton.points[i].x;
                    pcl_point.y =  single_skeleton.points[i].y;
                    pcl_point.z =  7.0;
                    pcl_point.r = r; pcl_point.g = g; pcl_point.b = b;
                    tmp_cloud_cluster->points.push_back(pcl_point);
                    cloud_cluster->points.push_back(pcl_point);
                  }
                }
              }
            }
            color_code+=color_code;
            cloud_clusters.push_back(tmp_cloud_cluster);
          }
        }
      }
    
    //std::cout << "skeletal cloud size: " << skeleton_cloud->points.size() << std::endl;  
    extract_skeletal_clusters(cloud_cluster, cloud_clusters);

    return true;
  }


  /**
   * @brief extract clusters with its centers from the skeletal cloud 
   * 
   */
  void extract_skeletal_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters) {
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

      float room_diff_thres = 3.0; 
      if(room_length.x < 0.5 || room_length.y < 0.5) {
        i++;
        continue;
      } else if (room_length.x > 6.0 /*&& room_length.y < 3.0*/) {
         //std::cout << "found y corridor" << std::endl; 
         //k_means_clustering(cloud_cluster);
         sensor_msgs::PointCloud2 cloud_cluster_msg;
         pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);
         cloud_cluster_msg.header.stamp = ros::Time::now();
         cloud_cluster_msg.header.frame_id = "map";
         cluster_cloud_pub_.publish(cloud_cluster_msg);
         
         bool found_all_planes = false;
         s_graphs::PlaneData y_plane1, y_plane2;
         found_all_planes = get_corridor_planes(plane_class::Y_VERT_PLANE, p1, p2, y_plane1, y_plane2);
         if(found_all_planes) {
            geometry_msgs::Point room_center = get_corridor_center(plane_class::Y_VERT_PLANE, p1, p2, y_plane1, y_plane2);
            correct_plane_d(plane_class::Y_VERT_PLANE, y_plane1);
            correct_plane_d(plane_class::Y_VERT_PLANE, y_plane2);
            s_graphs::RoomData room_candidate;
            room_candidate.id = i;
            room_candidate.room_length = room_length;
            room_candidate.room_center = room_center;
            room_candidate.x_planes.push_back(y_plane1); room_candidate.x_planes.push_back(y_plane2);
            room_candidates_vec.push_back(room_candidate);
            i++;
          } else {
            i++;
            continue; 
          }

      } else if (room_length.y > 6.0 /*&& room_length.x < 3.0*/) {
         //std::cout << "found x corridor" << std::endl; 
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_post = axis_clustering(cloud_cluster);
         sensor_msgs::PointCloud2 cloud_cluster_msg;
         pcl::toROSMsg(*cluster_cloud_post, cloud_cluster_msg);
         cloud_cluster_msg.header.stamp = ros::Time::now();
         cloud_cluster_msg.header.frame_id = "map";
         cluster_cloud_pub_.publish(cloud_cluster_msg);
         
         //k_means_clustering(cloud_cluster);
         bool found_all_planes = false;
         s_graphs::PlaneData x_plane1, x_plane2;
         found_all_planes = get_corridor_planes(plane_class::X_VERT_PLANE, p1, p2, x_plane1, x_plane2);
         correct_plane_d(plane_class::X_VERT_PLANE, x_plane1);
         correct_plane_d(plane_class::X_VERT_PLANE, x_plane2); 
         if(found_all_planes) {
            geometry_msgs::Point room_center = get_corridor_center(plane_class::X_VERT_PLANE, p1, p2, x_plane1, x_plane2);
            s_graphs::RoomData room_candidate;
            room_candidate.id = i;
            room_candidate.room_length = room_length;
            room_candidate.room_center = room_center;
            room_candidate.y_planes.push_back(x_plane1); room_candidate.y_planes.push_back(x_plane2);
            room_candidates_vec.push_back(room_candidate);
            i++;
          } else {
            i++;
            continue; 
          }

      } else {
          bool found_all_planes = false;
          //extract_line_segments(cloud_cluster);

          s_graphs::PlaneData x_plane1, x_plane2, y_plane1, y_plane2;
          found_all_planes = get_room_planes(p1,p2, x_plane1, x_plane2, y_plane1, y_plane2);
          correct_plane_d(plane_class::X_VERT_PLANE, x_plane1);
          correct_plane_d(plane_class::X_VERT_PLANE, x_plane2);
          correct_plane_d(plane_class::Y_VERT_PLANE, y_plane1);
          correct_plane_d(plane_class::Y_VERT_PLANE, y_plane2);

          if(found_all_planes) {
            geometry_msgs::Point room_center = get_room_center(p1, p2, x_plane1, x_plane2, y_plane1, y_plane2);
            s_graphs::RoomData room_candidate;
            room_candidate.id = i;
            room_candidate.room_length = room_length;
            room_candidate.room_center = room_center;
            room_candidate.x_planes.push_back(x_plane1); room_candidate.x_planes.push_back(x_plane2);
            room_candidate.y_planes.push_back(y_plane1); room_candidate.y_planes.push_back(y_plane2);
            room_candidates_vec.push_back(room_candidate);
            i++;
          }else {
            i++;
            continue;
          }
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
    //cluster_cloud_pub_.publish(cloud_cluster_msg);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr axis_clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud) {
    pcl::PointXYZRGB rand_number = skeleton_cloud->points[rand() % skeleton_cloud->points.size()];
    x_cluster_point x_centroid(rand_number.x, rand_number.y); x_centroid.cluster_id = 0;
    y_cluster_point y_centroid(rand_number.x, rand_number.y); y_centroid.cluster_id = 1;
    std::vector<cluster_point>  points_vec; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for(int i=0; i < skeleton_cloud->points.size(); ++i) {
      cluster_point point(skeleton_cloud->points[i].x, skeleton_cloud->points[i].y);
      points_vec.push_back(point);
    }     

    int num_epochs = 0;
    while(num_epochs < 100) {
        for(int i=0; i < points_vec.size(); ++i) {
          double dist = x_centroid.distance(points_vec[i]);
          if(dist < points_vec[i].min_dist) {
            points_vec[i].min_dist = dist;
            points_vec[i].cluster_id = x_centroid.cluster_id;
          }
        }

        for(int i=0; i < points_vec.size(); ++i) {
          double dist = y_centroid.distance(points_vec[i]);
          if(dist < points_vec[i].min_dist) {
            points_vec[i].min_dist = dist;
            points_vec[i].cluster_id = y_centroid.cluster_id;
          }
        }

        std::vector<int> num_points; 
        std::vector<double> sum_x; std::vector<double> sum_y;
        for(int i=0; i< 2; i++){
          num_points.push_back(0); 
          sum_x.push_back(0); 
          sum_y.push_back(0);
        }
        
       for(int i=0; i < points_vec.size(); ++i) {
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

    for(int i=0; i< points_vec.size(); ++i) {
      int cluster_id = points_vec[i].cluster_id;
      if(cluster_id == 0) {
        pcl::PointXYZRGB point;
        point.x = points_vec[i].x;
        point.y = points_vec[i].y;
        point.z = 7;
        point.r = 255;     
        point.g = 0;         
        point.b = 0;         
    
        cluster_cloud->points.push_back(point);
      }
      if(cluster_id == 1) {
        pcl::PointXYZRGB point;
        point.x = points_vec[i].x;
        point.y = points_vec[i].y;
        point.z = 7;
        point.r = 0;     
        point.g = 0;         
        point.b = 255;         
    
        cluster_cloud->points.push_back(point);
      }
    }
    std::cout << "x centroid: " << x_centroid.x << " ; " << x_centroid.y << std::endl;
    std::cout << "y centroid: " << y_centroid.x << " ; " << y_centroid.y << std::endl;
    return cluster_cloud;

  }

  void euclidean_clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud) {
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // tree->setInputCloud (skeleton_cloud);
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance (0.1); 
    // ec.setMinClusterSize (2);
    // ec.setMaxClusterSize (500);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (skeleton_cloud);
    // ec.extract (cluster_indices);

    // std::cout << "cloud indices size: " << cluster_indices.size() << std::endl;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;

    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    //   float r = rand()%256 ; float g = rand()%256; float b = rand()%256;
    //   for (const auto& idx : it->indices) {
    //     pcl::PointXYZRGB pointrgb;
    //     pointrgb.x = skeleton_cloud->points[idx].x;
    //     pointrgb.y = skeleton_cloud->points[idx].y;
    //     pointrgb.z = skeleton_cloud->points[idx].z;
    //     pointrgb.r = r;
    //     pointrgb.g = g;
    //     pointrgb.b = b;
    //     tmp_cloud_cluster->points.push_back(pointrgb);
    //   }
    //   cloud_clusters.push_back(tmp_cloud_cluster);
    // }

    // for(const auto& cloud_cluster_1 :  cloud_clusters) {
      
    //   for()
    // }

  }

  void k_means_clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster) {
    pcl::Kmeans k_means_clustering(static_cast<int> (cloud_cluster->points.size()), 3);
    k_means_clustering.setClusterSize(2);
    for (size_t i = 0; i < cloud_cluster->points.size(); i++) {
            std::vector<float> data(3);
            data[0] = cloud_cluster->points[i].x;
            data[1] = cloud_cluster->points[i].y;
            data[2] = cloud_cluster->points[i].z;
            k_means_clustering.addDataPoint(data);
    }

    k_means_clustering.kMeans();
    pcl::Kmeans::Centroids centroids = k_means_clustering.get_centroids();
    std::cout << "centroid count: " << centroids.size() << std::endl;
    for (int i = 0; i<centroids.size(); i++) {
        std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
        std::cout << "y: " << centroids[i][1] << " ,";
        std::cout << "z: " << centroids[i][2] << std::endl;
    }

  }

  void extract_line_segments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster) {
    while (cloud_cluster->size() > 10) {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      // Optional
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.5);
      seg.setInputCloud(cloud_cluster);
      seg.segment(*inliers, *coefficients);
      std::cout << "Model coeffs size: " << coefficients->values.size() << std::endl;
      std::cout << "Model coefficients before: " << coefficients->values[0] << "; " << coefficients->values[1] << "; " << coefficients->values[2] << "; " << coefficients->values[3] <<  "; " << coefficients->values[4] << "; " << coefficients->values[5] << std::endl;
      
      extract.setInputCloud(cloud_cluster);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*cloud_cluster);
    }
  }  
  

  void cluster_endpoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointXY& p1, pcl::PointXY& p2) {
    //pcl::PointXYZRGB pmin, pmax;
    //pcl::getMaxSegment(*cloud_cluster, pmin, pmax);
    //p1.x = pmin.x; p1.y = pmin.y;       
    //p2.x = pmax.x; p2.y = pmax.y;
    //float length = pcl::euclideanDistance(p1,p2);

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

    p1.x = min_x; p1.y = min_y;
    p2.x = max_x; p2.y = max_y;  

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
    // if(fabs(p1.x) > fabs(p2.x)) {
    //   float size = p1.x - p2.x;
    //   center.x = (size/2) + p2.x;
    // }else {
    //   float size = p2.x - p1.x;
    //   center.x = (size/2) + p1.x;
    // }
    
    // if(fabs(p1.y) > fabs(p2.y)) {
    //   float size = p1.y - p2.y;
    //   center.y = (size/2) + p2.y;
    // }else {
    //   float size = p2.y - p1.y;
    //   center.y = (size/2) + p1.y;
    // }

    if(fabs(x_plane1.d) > fabs(x_plane2.d)) {
        double size = x_plane1.d - x_plane2.d;
        center.x = (((size)/2) + x_plane2.d);
    } else {
        double size = x_plane2.d - x_plane1.d;
        center.x = (((size)/2) + x_plane1.d);
    }

    if(fabs(y_plane1.d) > fabs(y_plane2.d)) {
        double size = y_plane1.d - y_plane2.d;
        center.y = (((size)/2) + y_plane2.d);
    } else {
        double size = y_plane2.d - y_plane1.d;
        center.y = (((size)/2) + y_plane1.d);
    }


    return center;
  }


  geometry_msgs::Point get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2) {
    geometry_msgs::Point center; 

    if(plane_type == plane_class::X_VERT_PLANE){
      if(fabs(plane1.d) > fabs(plane2.d)) {
        double size = plane1.d - plane2.d;
        center.x = (((size)/2) + plane2.d);
     } else {
        double size = plane2.d - plane1.d;
        center.x = (((size)/2) + plane1.d);
     }
     
      if(fabs(p1.y) > fabs(p2.y)) {
        float size = p1.y - p2.y;
        center.y = (size/2) + p2.y;
      } else {
        float size = p2.y - p1.y;
        center.y = (size/2) + p1.y;
      }
    }

    if(plane_type == plane_class::Y_VERT_PLANE){
      if(fabs(plane1.d) > fabs(plane2.d)) {
        double size = plane1.d - plane2.d;
        center.y = (((size)/2) + plane2.d);
     } else {
        double size = plane2.d - plane1.d;
        center.y = (((size)/2) + plane1.d);
     }
     
      if(fabs(p1.x) > fabs(p2.x)) {
        float size = p1.x - p2.x;
        center.x = (size/2) + p2.x;
      } else {
        float size = p2.x - p1.x;
        center.x = (size/2) + p1.x;
      }
    }

    return center;
  }



  void correct_plane_d(int plane_type, s_graphs::PlaneData& plane) {
    
    if(plane_type == plane_class::X_VERT_PLANE){
      plane.d = -1*plane.d;
      double p_norm = plane.nx/fabs(plane.nx);
      plane.d = p_norm*plane.d; 
    }
    
    if(plane_type == plane_class::Y_VERT_PLANE){
      plane.d = -1*plane.d;
      double p_norm = plane.ny/fabs(plane.ny);
      plane.d = p_norm*plane.d; 
    }
  }

  bool get_corridor_planes(int plane_type, pcl::PointXY p_min, pcl::PointXY p_max, s_graphs::PlaneData& plane1, s_graphs::PlaneData& plane2) {
    float corridor_dist_thres = 1.5;
    float min_dist_p1 =100; float min_dist_p2 =100;
    bool found_plane1 = false, found_plane2 = false;

    if(plane_type == plane_class::X_VERT_PLANE) {
      for(const auto& x_plane : x_vert_plane_vec) {
      //std::cout << "xplane1: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
      if(x_plane.nx < 0) {
        continue;
      }
      float dist_x1 = -1*(x_plane.nx * p_min.x + x_plane.ny * p_min.y);
      float diff_dist_x1 = 100;
      diff_dist_x1 = fabs(dist_x1 - x_plane.d);

      //std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;
      if(diff_dist_x1 < min_dist_p1) {
        min_dist_p1 = diff_dist_x1;
        plane1 = x_plane;
      }
     }

      for(const auto& x_plane : x_vert_plane_vec) {
        //std::cout << "xplane2: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
        if(x_plane.nx > 0) {
          continue;
        }
        float dist_x2 = -1*(x_plane.nx * p_max.x + x_plane.ny * p_max.y);
        float diff_dist_x2 = 100;
        diff_dist_x2 = fabs(dist_x2 - x_plane.d);
        
        //std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;
        if(diff_dist_x2 < min_dist_p2) {
          min_dist_p2 = diff_dist_x2;
          plane2 = x_plane;
        }
      }
    
    //std::cout << "selected xplane1 : " << plane1.nx << ", " << plane1.ny << ", " << plane1.nz << ", " << plane1.d << std::endl;
    //std::cout << "selected xplane2 : " << plane2.nx << ", " << plane2.ny << ", " << plane2.nz << ", " << plane2.d << std::endl;

    if(plane1.nx * plane2.nx > 0) {
      //std::cout << "no xplane1 found " << std::endl;    
      //std::cout << "no xplane2 found " << std::endl;
      found_plane1 = false, found_plane2 = false;    
    } else {
      if(min_dist_p1 < corridor_dist_thres) {
        //std::cout << "corridor has xplane1: " << plane1.nx << ", " << plane1.ny << ", " << plane1.nz << ", " << plane1.d << std::endl;
        found_plane1 = true;
      }
      else  {
        //std::cout << "no xplane1 found " << std::endl;  
        found_plane1 = false;
      }
      if(min_dist_p2 < corridor_dist_thres) {
        //std::cout << "corridor has xplane2: " << plane2.nx << ", " << plane2.ny << ", " << plane2.nz << ", " << plane2.d << std::endl;  
        found_plane2 = true;
      }  
      else {
        //std::cout << "no xplane2 found " << std::endl;   
        found_plane2 = false;
      }
    }
  }


  if(plane_type == plane_class::Y_VERT_PLANE) {
    for(const auto& y_plane : y_vert_plane_vec) {
      //std::cout << "yplane1: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
      if(y_plane.ny < 0) {
        continue;
      }
      float dist_y1 = -1*(y_plane.nx * p_min.x + y_plane.ny * p_min.y);
      float diff_dist_y1 = 100;
      diff_dist_y1 = fabs(dist_y1 - y_plane.d);

      //std::cout << "diff dist x1: " << diff_dist_y1 << std::endl;
      if(diff_dist_y1 < min_dist_p1) {
        min_dist_p1 = diff_dist_y1;
        plane1 = y_plane;
      }
    }
  
    for(const auto& y_plane : y_vert_plane_vec) {
      //std::cout << "yplane2: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
      if(y_plane.ny > 0) {
        continue;
      }
      float dist_y2 = -1*(y_plane.nx * p_max.x + y_plane.ny * p_max.y);
      float diff_dist_y2 = 100;
      diff_dist_y2 = fabs(dist_y2 - y_plane.d);

      //std::cout << "diff dist x1: " << diff_dist_y2 << std::endl;
      if(diff_dist_y2 < min_dist_p1) {
        min_dist_p1 = diff_dist_y2;
        plane1 = y_plane;
      }
    }

        
    //std::cout << "selected yplane1 : " << plane1.nx << ", " << plane1.ny << ", " << plane1.nz << ", " << plane1.d << std::endl;
    //std::cout << "selected yplane2 : " << plane2.nx << ", " << plane2.ny << ", " << plane2.nz << ", " << plane2.d << std::endl;

    if(plane1.nx * plane2.nx > 0) {
      //std::cout << "no yplane1 found " << std::endl;    
      //std::cout << "no yplane2 found " << std::endl;
      found_plane1 = false, found_plane2 = false;    
    } else {
      if(min_dist_p1 < corridor_dist_thres) {
        //std::cout << "corridor has yplane1: " << plane1.nx << ", " << plane1.ny << ", " << plane1.nz << ", " << plane1.d << std::endl;
        found_plane1 = true;
      }
      else  {
        //std::cout << "no yplane1 found " << std::endl;  
        found_plane1 = false;
      }
      if(min_dist_p2 < corridor_dist_thres) {
        //std::cout << "corridor has yplane2: " << plane2.nx << ", " << plane2.ny << ", " << plane2.nz << ", " << plane2.d << std::endl;  
        found_plane2 = true;
      }  
      else {
        //std::cout << "no yplane2 found " << std::endl;   
        found_plane2 = false;
      }
    }
  }

  if(found_plane1 && found_plane2)
    return true;
  else 
    return false;

  }

  bool get_room_planes(pcl::PointXY p_min, pcl::PointXY p_max, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2) {
    float room_dist_thres = 1.5;
    float min_dist_x1 =100; float min_dist_x2 =100;
    bool found_x1_plane = false, found_x2_plane = false, found_y1_plane = false, found_y2_plane = false;

    for(const auto& x_plane : x_vert_plane_vec) {
      if(x_plane.nx < 0) {
        continue;
      }

      //std::cout << "xplane1: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
      float dist_x1 = -1*(x_plane.nx * p_min.x + x_plane.ny * p_min.y);
      float diff_dist_x1 = 100;
      diff_dist_x1 = fabs(dist_x1 - x_plane.d);

      //std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;
      if(diff_dist_x1 < min_dist_x1) {
        min_dist_x1 = diff_dist_x1;
        x_plane1 = x_plane;
      }
    }

    for(const auto& x_plane : x_vert_plane_vec) {
      if(x_plane.nx > 0) {
        continue;
      }

      //std::cout << "xplane2: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
      float dist_x2 = -1*(x_plane.nx * p_max.x + x_plane.ny * p_max.y);
      float diff_dist_x2 = 100;
      diff_dist_x2 = fabs(dist_x2 - x_plane.d);
      
      //std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;
      if(diff_dist_x2 < min_dist_x2) {
        min_dist_x2 = diff_dist_x2;
        x_plane2 = x_plane;
      }
    }

    
    //std::cout << "selected xplane1 : " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
    //std::cout << "selected xplane2 : " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;

    if(x_plane1.nx * x_plane2.nx > 0) {
      //std::cout << "no xplane1 found " << std::endl;    
      //std::cout << "no xplane2 found " << std::endl;
      found_x1_plane = false, found_x2_plane = false;    
    } else {
      if(min_dist_x1 < room_dist_thres) {
        //std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
        found_x1_plane = true;
      }
      else  {
        //std::cout << "no xplane1 found " << std::endl;  
        found_x1_plane = false;
      }
      if(min_dist_x2 < room_dist_thres) {
        //std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;  
        found_x2_plane = true;
      }  
      else {
        //std::cout << "no xplane2 found " << std::endl;   
        found_x2_plane = false;
      }
    }

    float min_dist_y1 =100; float min_dist_y2 =100;
    for(const auto& y_plane : y_vert_plane_vec) {
      if(y_plane.ny < 0) {
        continue; 
      }
      //std::cout << "y_plane1: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;

      float dist_y1 = -1*(y_plane.nx * p_min.x + y_plane.ny * p_min.y);
      float diff_dist_y1 = 100;
      diff_dist_y1 = fabs(dist_y1 - y_plane.d);

      //std::cout << "diff dist y1: " << diff_dist_y1 << std::endl;
      if(diff_dist_y1 < min_dist_y1) {
        min_dist_y1 = diff_dist_y1;
        y_plane1 = y_plane;
      }
    }

    for(const auto& y_plane : y_vert_plane_vec) {
      if(y_plane.ny > 0) {
        continue;
      }
      //std::cout << "y_plane2: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;

      float dist_y2 = -1*(y_plane.nx * p_max.x + y_plane.ny * p_max.y);
      float diff_dist_y2 = 100;
      diff_dist_y2 = fabs(dist_y2 - y_plane.d);

      //std::cout << "diff dist y2: " << diff_dist_y2 << std::endl;
      if(diff_dist_y2 < min_dist_y2) {
        min_dist_y2 = diff_dist_y2;
        y_plane2 = y_plane;
      }
    } 

    //std::cout << "selected yplane1 : " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
    //std::cout << "selected yplane2 : " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;

    if(y_plane1.ny * y_plane2.ny > 0) {
      //std::cout << "no yplane1 found " << std::endl;    
      //std::cout << "no yplane2 found " << std::endl;  
      found_y1_plane = false, found_y2_plane = false;
    } else {
      if(min_dist_y1 < room_dist_thres) {
        //std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
        found_y1_plane = true;
      }
      else {
        //std::cout << "no yplane1 found " << std::endl;    
        found_y1_plane = false;
      }
      if(min_dist_y2 < room_dist_thres) {
        //std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;  
        found_y2_plane = true;
      } 
      else {
        //std::cout << "no yplane2 found " << std::endl;   
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

  struct cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    cluster_point() : 
        x(0.0), 
        y(0.0),
        cluster_id(-1),
        min_dist(1000) {}
        
    cluster_point(double x, double y) : 
        x(x), 
        y(y),
        cluster_id(-1),
        min_dist(1000) {}
  };
  

  struct x_cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    x_cluster_point() : 
        x(0.0), 
        y(0.0),
        cluster_id(-1),
        min_dist(1000) {}
        
    x_cluster_point(double x, double y) : 
        x(x), 
        y(y),
        cluster_id(-1),
        min_dist(1000) {}

    double distance(cluster_point p) {
      return (p.x - x) * (p.x - x);
    }
  };

  struct y_cluster_point {
    int cluster_id;
    double x;
    double y;
    double min_dist;

    y_cluster_point() : 
        x(0.0), 
        y(0.0),
        cluster_id(-1),
        min_dist(1000) {}
        
    y_cluster_point(double x, double y) : 
        x(x), 
        y(y),
        cluster_id(-1),
        min_dist(1000) {}

    double distance(cluster_point p) {
      return (p.y - y) * (p.y - y);
    }
  };


  /* private variables */
private:
  //skeleton graph queue
  std::deque<visualization_msgs::MarkerArray::Ptr> skeleton_graph_queue; 
  std::vector<s_graphs::PlaneData> x_vert_plane_vec, y_vert_plane_vec;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  enum plane_class : uint8_t{
    X_VERT_PLANE = 0,
    Y_VERT_PLANE = 1,
  };
};

}  // namespace room segmentation

PLUGINLIB_EXPORT_CLASS(s_graphs::RoomSegmentationNodelet, nodelet::Nodelet)
