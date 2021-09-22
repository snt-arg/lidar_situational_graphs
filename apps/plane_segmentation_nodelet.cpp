#include <iostream>
#include <string>
#include <ros/time.h>
#include <hdl_graph_slam/plane_segmentor.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
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
    filtered_point_cloud_sub_ = nh.subscribe("filtered_point_cloud", 1, &PlaneSegmentationNodelet::filteredPointCloudCallback, this);
  }

  void filteredPointCloudCallback(const pcl::PointCloud<PointT>::ConstPtr& src_cloud) {
    if(src_cloud->empty()) {
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
    }
  }

  void segment_planes(pcl::PointCloud<PointT>::Ptr transformed_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(transformed_cloud);
    seg.segment(*inliers, *coefficients);

    std::cout << inliers->indices.size() << std::endl;
  }

private:
  ros::Subscriber filtered_point_cloud_sub_;

  /* private variables */
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  tf::TransformListener tf_listener;
  std::string base_link_frame;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PlaneSegmentationNodelet, nodelet::Nodelet)
