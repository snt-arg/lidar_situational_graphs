#include <iostream>
#include <string>
#include <ros/time.h>
#include <hdl_graph_slam/plane_segmentor.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

namespace hdl_graph_slam {

class PlaneSegmentationNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZRGB PointT;

  PlaneSegmentationNodelet() {}
  virtual ~PlaneSegmentationNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    this->init_ros();
  }

private:
  void init_ros() {
    filtered_point_cloud_sub_ = nh.subscribe("filtered_point_cloud", 1, &PlaneSegmentationNodelet::filteredPointCloudCallback, this);
  }
  
  void filteredPointCloudCallback(const pcl::PointCloud<PointT>::ConstPtr& msg) {
    return;
  }

private:
  ros::Subscriber filtered_point_cloud_sub_;

  /* private variables */
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PlaneSegmentationNodelet, nodelet::Nodelet)
