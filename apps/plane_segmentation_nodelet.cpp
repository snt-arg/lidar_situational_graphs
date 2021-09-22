#include <hdl_graph_slam/plane_segmentor.hpp>
#include <nodelet/nodelet.h>

#include <iostream>


namespace hdl_graph_slam {

class plane_segmentation_nodelet : public nodelet::Nodelet

public:
plane_segmentation_nodelet() {}
~plane_segmentation_nodelet() {}


/* publisher and subscribers */
ros::Subscriber filtered_point_cloud_sub_;
void filteredPointCloudCallback(const sensor_msgs::PointCloud2& msg);

}

