#include <iostream>
#include <string>
#include <ros/time.h>
#include <hdl_graph_slam/plane_segmentor.hpp>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

namespace hdl_graph_slam {

class PlaneSegmentationNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZRGB PointT;

  PlaneSegmentationNodelet() {}
  virtual ~PlaneSegmentationNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    this->initialize_params();
    this->init_ros();
  }

private:
  void initialize_params() {
    base_link_frame = private_nh.param<std::string>("base_link_frame", "base_link");
  }

  void init_ros() {
    filtered_point_cloud_sub_ = nh.subscribe("velodyne_points", 64, &PlaneSegmentationNodelet::filteredPointCloudCallback, this);
    segmented_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("segmented_points", 1);
  }

  void filteredPointCloudCallback(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) {
    if(src_cloud->empty()) {
      std::cout << "Plane Segmentation got empty point cloud" << std::endl;
      return;
    }

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!base_link_frame.empty()) {
      if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
      }

      tf::StampedTransform transform;
      tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = src_cloud->header.stamp;

      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
      normal_cloud = this->compute_normals(transformed);
      this->segment_planes(transformed, normal_cloud);
    }
  }

  pcl::PointCloud<pcl::Normal>::Ptr compute_normals(pcl::PointCloud<PointT>::Ptr transformed_cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    normal_cloud->clear();

    // pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    // ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    // ne.setMaxDepthChangeFactor(0.03f);
    // ne.setNormalSmoothingSize(20.0f);
    // pcl::NormalEstimation<PointT, pcl::Normal> ne;
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    // ne.setSearchMethod(tree);
    // ne.setInputCloud(transformed_cloud);
    // ne.setRadiusSearch (0.03);
    // // ne.setIndices(inliers);
    // ne.compute(*normal_cloud);
    // std::cout << normal_cloud->size() << std::endl;
    return normal_cloud;
  }

  void segment_planes(pcl::PointCloud<PointT>::Ptr transformed_cloud, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud) {
    pcl::PointCloud<PointT> segmented_cloud;
    for(int i = 0; i < 4; ++i) {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Optional
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.1);
      if(i == 0)
        seg.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));
      else if(i == 1)
        seg.setAxis(Eigen::Vector3f(-1.0, 0.0, 0.0));
      else if(i == 2)
        seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
      else if(i == 3)
        seg.setAxis(Eigen::Vector3f(0.0, -1.0, 0.0));
      seg.setEpsAngle(pcl::deg2rad(3.0f));
      seg.setInputCloud(transformed_cloud);
      // seg.setInputNormals(normal_cloud);
      seg.segment(*inliers, *coefficients);

      for(const auto& idx : inliers->indices) {
        segmented_cloud.points.push_back(transformed_cloud->points[idx]);
        int index = this->getIndex(transformed_cloud->points, transformed_cloud->points[idx]);
        //std::cout << "index: " << index << std::endl;
        //transformed_cloud->points.erase(transformed_cloud->points.begin() + index);
        if(i == 0) {
          segmented_cloud.back().r = 255;
          segmented_cloud.back().g = 0;
          segmented_cloud.back().b = 0;
        } else if(i == 1) {
          segmented_cloud.back().r = 0;
          segmented_cloud.back().g = 0;
          segmented_cloud.back().b = 255;
        } else if(i == 2) {
          segmented_cloud.back().r = 0;
          segmented_cloud.back().g = 255;
          segmented_cloud.back().b = 0;
        } else if(i == 3) {
          segmented_cloud.back().r = 255;
          segmented_cloud.back().g = 192;
          segmented_cloud.back().b = 204;
        }
      }
    }

    sensor_msgs::PointCloud2 segmented_cloud_msg;
    pcl::toROSMsg(segmented_cloud, segmented_cloud_msg);
    std_msgs::Header msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
    segmented_cloud_msg.header = msg_header;
    segmented_cloud_msg.header.frame_id = "velodyne";  // this is temporary
    segmented_cloud_pub_.publish(segmented_cloud_msg);
  }

  int getIndex(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > v, pcl::PointXYZRGB K) {
    // auto it = std::find(v.begin(), v.end(), K);
    auto it = find_if(v.begin(), v.end(), [&K](const pcl::PointXYZRGB& obj) { return (obj.x == K.x && obj.y == K.y && obj.z == K.z); });
    int index = std::distance(v.begin(), it);
    return index;
  }

private:
  ros::Subscriber filtered_point_cloud_sub_;
  ros::Publisher segmented_cloud_pub_;

  /* private variables */
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  tf::TransformListener tf_listener;
  std::string base_link_frame;
  friend bool operator==(const PointT& p1, const PointT& p2);
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PlaneSegmentationNodelet, nodelet::Nodelet)
