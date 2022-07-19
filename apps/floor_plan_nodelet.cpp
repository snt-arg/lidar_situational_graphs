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
  void initialize_params() {}

  void init_ros() {
    skeleton_graph_sub = nh.subscribe("/voxblox_skeletonizer/sparse_graph", 1, &FloorPlanNodelet::skeleton_graph_callback, this);
    // map_planes_sub = nh.subscribe("/s_graphs/complete_map_planes", 100, &FloorPlanNodelet::map_planes_callback, this);

    cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/room_segmentation/cluster_cloud", 1, true);
  }

  /**
   *
   * @brief get the points from the skeleton graph for clusterting and identifying room candidates
   * @param skeleton_graph_msg
   */
  void skeleton_graph_callback(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {}

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::Subscriber skeleton_graph_sub;
  ros::Subscriber map_planes_sub;
  ros::Publisher cluster_cloud_pub_;
};

}  // namespace s_graphs
PLUGINLIB_EXPORT_CLASS(s_graphs::FloorPlanNodelet, nodelet::Nodelet)
