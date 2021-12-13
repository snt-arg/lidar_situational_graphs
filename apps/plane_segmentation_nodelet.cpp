#include <iostream>
#include <string>
#include <cmath>
#include <math.h>

#include <hdl_graph_slam/PointClouds.h>

#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
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
namespace hdl_graph_slam {

class PlaneSegmentationNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZRGBNormal PointT;

  PlaneSegmentationNodelet() : cloud_accumulated_(new pcl::PointCloud<PointT>()) {}
  virtual ~PlaneSegmentationNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    this->initialize_params();
    this->init_ros();
  }

private:
  void initialize_params() {
    this->cloud_accum_ = 0;
    plane_extraction_frame_ = private_nh.param<std::string>("plane_extraction_frame_id", "base_link");
    min_seg_points_ = private_nh.param<int>("min_seg_points", 1000);
    min_horizontal_inliers_ = private_nh.param<int>("min_horizontal_inliers", 800);
    min_vertical_inliers_ = private_nh.param<int>("min_vertical_inliers", 500);
    use_euclidean_filter_ = private_nh.param<bool>("use_euclidean_filter", true);
    use_shadow_filter_ = private_nh.param<bool>("use_shadow_filter", false);

  }

  void init_ros() {
    filtered_point_cloud_sub_ = nh.subscribe("velodyne_points", 64, &PlaneSegmentationNodelet::filteredPointCloudCallback, this);
    segmented_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    segmented_clouds_pub_ = nh.advertise<hdl_graph_slam::PointClouds>("segmented_clouds", 1);
  }

  void filteredPointCloudCallback(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) {
    if(src_cloud->empty()) {
      std::cout << "Plane Segmentation got empty point cloud" << std::endl;
      return;
    }

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!plane_extraction_frame_.empty()) {
      if(!tf_listener_.canTransform(plane_extraction_frame_, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << plane_extraction_frame_ << " and " << src_cloud->header.frame_id << std::endl;
      }
      tf::StampedTransform transform;
      tf_listener_.waitForTransform(plane_extraction_frame_, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener_.lookupTransform(plane_extraction_frame_, src_cloud->header.frame_id, ros::Time(0), transform);

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = plane_extraction_frame_;
      transformed->header.stamp = src_cloud->header.stamp;
      // pcl::copyPointCloud(*src_cloud, *transformed);
      this->segment_planes(transformed);
    }
  }

  void segment_planes(pcl::PointCloud<PointT>::Ptr transformed_cloud) {
    pcl::PointCloud<PointT>::Ptr segmented_cloud(new pcl::PointCloud<PointT>);
    std::vector<sensor_msgs::PointCloud2> extracted_cloud_vec;
    int i = 0;
    while(transformed_cloud->points.size() > min_seg_points_) {
      try {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ExtractIndices<PointT> extract;
        // Optional
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        // seg.setEpsAngle(pcl::deg2rad(5.0f));
        seg.setInputCloud(transformed_cloud);
        // seg.setInputNormals(normal_cloud);
        int model_type = seg.getModelType();
        seg.segment(*inliers, *coefficients);
        /* check if indicies are not empty for no crash */
        if(inliers->indices.empty()) {
          std::cout << "Breaking as no model found" << std::endl;
          break;
        }
        /* filtering out noisy ground plane measurements */
        if((fabs(coefficients->values[2]) > 0.9 && inliers->indices.size() < min_horizontal_inliers_) || inliers->indices.size() < min_vertical_inliers_) {
          extract.setInputCloud(transformed_cloud);
          extract.setIndices(inliers);
          extract.setNegative(true);
          extract.filter(*transformed_cloud);
          continue;
        }

        Eigen::Vector4d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
        Eigen::Vector3d closest_point = normal.head(3) * normal(3);
        Eigen::Vector4d plane;
        plane.head(3) = closest_point / closest_point.norm();
        plane(3) = closest_point.norm();

        // std::cout << "Model coefficients before " << std::to_string(i) << ": " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
        // std::cout << "Model coefficients after " << std::to_string(i) << ": " << plane << std::endl;

        pcl::PointCloud<PointT>::Ptr extracted_cloud(new pcl::PointCloud<PointT>);
        for(const auto& idx : inliers->indices) {
          extracted_cloud->push_back(transformed_cloud->points[idx]);
          extracted_cloud->back().normal_x = plane(0);
          extracted_cloud->back().normal_y = plane(1);
          extracted_cloud->back().normal_z = plane(2);
          extracted_cloud->back().curvature = plane(3);
        }
        
        pcl::PointCloud<PointT>::Ptr extracted_cloud_filtered;
        if(use_euclidean_filter_)
          extracted_cloud_filtered = compute_clusters(extracted_cloud);
        else if(use_shadow_filter_){
          pcl::PointCloud<pcl::Normal>::Ptr normals = compute_cloud_normals(extracted_cloud);
          extracted_cloud_filtered = shadow_filter(extracted_cloud, normals);
        } else {
          extracted_cloud_filtered = extracted_cloud;
        }

        // visulazing the pointcloud
        for(int pc = 0; pc < extracted_cloud_filtered->points.size(); ++pc) {
          segmented_cloud->points.push_back(extracted_cloud_filtered->points[pc]);
          segmented_cloud->back().r = 254;
          segmented_cloud->back().g = 216;
          segmented_cloud->back().b = 177;
        }

        sensor_msgs::PointCloud2 extracted_cloud_msg;
        pcl::toROSMsg(*extracted_cloud_filtered, extracted_cloud_msg);
        std_msgs::Header ext_msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
        extracted_cloud_msg.header = ext_msg_header;
        extracted_cloud_msg.header.frame_id = plane_extraction_frame_;
        extracted_cloud_vec.push_back(extracted_cloud_msg);

        extract.setInputCloud(transformed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*transformed_cloud);
        i++;
      } catch(const std::exception& e) {
        std::cout << "No ransac model found" << std::endl;
        break;
      }
    }

    hdl_graph_slam::PointClouds extracted_clouds_msg;
    std_msgs::Header ext_msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
    extracted_clouds_msg.header = ext_msg_header;
    extracted_clouds_msg.header.frame_id = plane_extraction_frame_;
    extracted_clouds_msg.pointclouds = extracted_cloud_vec;
    segmented_clouds_pub_.publish(extracted_clouds_msg);

    sensor_msgs::PointCloud2 segmented_cloud_msg;
    pcl::toROSMsg(*segmented_cloud, segmented_cloud_msg);
    std_msgs::Header msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
    segmented_cloud_msg.header = msg_header;
    segmented_cloud_msg.header.frame_id = plane_extraction_frame_;
    segmented_cloud_pub_.publish(segmented_cloud_msg);
  }

  pcl::PointCloud<PointT>::Ptr compute_clusters(const pcl::PointCloud<PointT>::Ptr& extracted_cloud) {
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(extracted_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(extracted_cloud);
    ec.extract(cluster_indices);

    //auto max_iterator = std::max_element(std::begin(cluster_indices), std::end(cluster_indices), [](const pcl::PointIndices& lhs, const pcl::PointIndices& rhs) { return lhs.indices.size() < rhs.indices.size(); });

    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (auto single_cluster : cluster_indices) {  
      for(const auto& idx : (single_cluster).indices) {
        cloud_cluster->push_back(extracted_cloud->points[idx]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->back().normal_x = extracted_cloud->back().normal_x;
        cloud_cluster->back().normal_y = extracted_cloud->back().normal_y;
        cloud_cluster->back().normal_z = extracted_cloud->back().normal_z;
        cloud_cluster->back().curvature = extracted_cloud->back().curvature;
      }
    }

    return cloud_cluster;
  }

  pcl::PointCloud<pcl::Normal>::Ptr compute_cloud_normals(const pcl::PointCloud<PointT>::Ptr& extracted_cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    ne.setInputCloud(extracted_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);

    return cloud_normals;
  }

  pcl::PointCloud<PointT>::Ptr shadow_filter(const pcl::PointCloud<PointT>::Ptr& extracted_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
    pcl::PointCloud<PointT>::Ptr extracted_cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::ShadowPoints<PointT, pcl::Normal> sp_filter;
    sp_filter.setNormals(cloud_normals);
    sp_filter.setThreshold(0.1);
    sp_filter.setInputCloud(extracted_cloud);
    sp_filter.filter(*extracted_cloud_filtered);

    return extracted_cloud_filtered;
  }

  int getIndex(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > v, pcl::PointXYZRGB K) {
    auto it = find_if(v.begin(), v.end(), [&K](const pcl::PointXYZRGB& obj) { return (obj.x == K.x && obj.y == K.y && obj.z == K.z); });
    int index = std::distance(v.begin(), it);
    return index;
  }

private:
  ros::Subscriber filtered_point_cloud_sub_;
  ros::Publisher segmented_cloud_pub_;
  ros::Publisher segmented_clouds_pub_;

  /* private variables */
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  int cloud_accum_;
  pcl::PointCloud<PointT>::Ptr cloud_accumulated_;
  tf::TransformListener tf_listener_;
  std::string plane_extraction_frame_;
  int min_seg_points_, min_horizontal_inliers_, min_vertical_inliers_;
  bool use_euclidean_filter_, use_shadow_filter_; 
  friend bool operator==(const PointT& p1, const PointT& p2);
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PlaneSegmentationNodelet, nodelet::Nodelet)
