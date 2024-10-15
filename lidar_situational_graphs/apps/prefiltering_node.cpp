/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <s_graphs/common/point_types.hpp>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace s_graphs {

class PrefilteringNode : public rclcpp::Node {
 public:
  PrefilteringNode() : Node("prefiltering_node") {
    initialize_params();

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    if (this->get_parameter("deskewing").get_parameter_value().get<bool>()) {
      imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
          "imu/data",
          1,
          std::bind(&PrefilteringNode::imu_callback, this, std::placeholders::_1));
    }

    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "velodyne_points",
        64,
        std::bind(&PrefilteringNode::cloud_callback, this, std::placeholders::_1));
    points_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 32);
    colored_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_points", 32);
  }

 private:
  void initialize_params() {
    this->declare_parameter("deskewing", false);
    this->declare_parameter("downsample_method", "VOXELGRID");
    this->declare_parameter("downsample_resolution", 0.1);
    this->declare_parameter("outlier_removal_method", "STATISTICAL");
    this->declare_parameter("statistical_mean_k", 20);
    this->declare_parameter("statistical_stddev", 1.0);
    this->declare_parameter("radius_radius", 0.8);
    this->declare_parameter("radius_min_neighbors", 2.0);
    this->declare_parameter("use_distance_filter", true);
    this->declare_parameter("distance_near_thresh", 1.0);
    this->declare_parameter("distance_far_thresh", 100.0);
    this->declare_parameter("base_link_frame", "");
    this->declare_parameter("scan_period", 0.1);

    std::string downsample_method = this->get_parameter("downsample_method")
                                        .get_parameter_value()
                                        .get<std::string>();
    double downsample_resolution = this->get_parameter("downsample_resolution")
                                       .get_parameter_value()
                                       .get<double>();

    if (downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      pcl::VoxelGrid<PointT>::Ptr voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(
          downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if (downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution
                << std::endl;
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(
          new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(
          downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if (downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")"
                  << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    std::string outlier_removal_method = this->get_parameter("outlier_removal_method")
                                             .get_parameter_value()
                                             .get<std::string>();
    if (outlier_removal_method == "STATISTICAL") {
      int mean_k =
          this->get_parameter("statistical_mean_k").get_parameter_value().get<int>();
      double stddev_mul_thresh =
          this->get_parameter("statistical_stddev").get_parameter_value().get<double>();
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - "
                << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(
          new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if (outlier_removal_method == "RADIUS") {
      double radius =
          this->get_parameter("radius_radius").get_parameter_value().get<double>();
      int min_neighbors = this->get_parameter("radius_min_neighbors")
                              .get_parameter_value()
                              .get<double>();
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors
                << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(
          new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter =
        this->get_parameter("use_distance_filter").get_parameter_value().get<bool>();
    distance_near_thresh =
        this->get_parameter("distance_near_thresh").get_parameter_value().get<double>();
    distance_far_thresh =
        this->get_parameter("distance_far_thresh").get_parameter_value().get<double>();

    base_link_frame =
        this->get_parameter("base_link_frame").get_parameter_value().get<std::string>();
    std::string ns = this->get_namespace();
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(this->get_namespace()).substr(1);
      base_link_frame = ns_prefix + "/" + base_link_frame;
    }
  }

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    imu_queue.push_back(imu_msg);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr src_cloud_msg) {
    pcl::PointCloud<PointT>::Ptr src_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*src_cloud_msg, *src_cloud);

    if (src_cloud->empty()) {
      return;
    }

    src_cloud = deskewing(src_cloud);

    // if base_link_frame is defined, transform the input cloud to the frame
    geometry_msgs::msg::TransformStamped transform_msg;
    if (!base_link_frame.empty()) {
      try {
        transform_msg = tf_buffer->lookupTransform(
            base_link_frame, src_cloud->header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_INFO(this->get_logger(),
                    "Could not transform %s to %s: %s",
                    base_link_frame.c_str(),
                    src_cloud->header.frame_id.c_str(),
                    ex.what());
        return;
      }

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform_msg);
      src_cloud = transformed;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered, filtered_msg);
    filtered_msg.header.stamp = src_cloud_msg->header.stamp;
    filtered_msg.header.frame_id = base_link_frame;
    points_pub->publish(filtered_msg);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(
      const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(
      const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(
      const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(),
                 cloud->end(),
                 std::back_inserter(filtered->points),
                 [&](const PointT& p) {
                   double d = p.getVector3fMap().norm();
                   return d > distance_near_thresh && d < distance_far_thresh;
                 });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr deskewing(const pcl::PointCloud<PointT>::Ptr& cloud) {
    rclcpp::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    if (imu_queue.empty()) {
      return cloud;
    }

    // the color encodes the point number in the point sequence
    if (colored_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      colored->header = cloud->header;
      colored->is_dense = cloud->is_dense;
      colored->width = cloud->width;
      colored->height = cloud->height;
      colored->resize(cloud->size());

      for (size_t i = 0; i < cloud->size(); i++) {
        double t = static_cast<double>(i) / cloud->size();
        colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
        colored->at(i).r = 255 * t;
        colored->at(i).g = 128;
        colored->at(i).b = 255 * (1 - t);
      }

      sensor_msgs::msg::PointCloud2 colored_msg;
      pcl::toROSMsg(*cloud, colored_msg);
      colored_pub->publish(colored_msg);
    }

    sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_queue.front();

    auto loc = imu_queue.begin();
    for (; loc != imu_queue.end(); loc++) {
      imu_msg = (*loc);

      if (rclcpp::Time((*loc)->header.stamp) > stamp) {
        break;
      }
    }

    imu_queue.erase(imu_queue.begin(), loc);

    Eigen::Vector3f ang_v(imu_msg->angular_velocity.x,
                          imu_msg->angular_velocity.y,
                          imu_msg->angular_velocity.z);
    ang_v *= -1;

    pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;
    deskewed->width = cloud->width;
    deskewed->height = cloud->height;
    deskewed->resize(cloud->size());

    double scan_period =
        this->get_parameter("scan_period").get_parameter_value().get<double>();
    for (size_t i = 0; i < cloud->size(); i++) {
      const auto& pt = cloud->at(i);

      // TODO: transform IMU data into the LIDAR frame
      double delta_t = scan_period * static_cast<double>(i) / cloud->size();
      Eigen::Quaternionf delta_q(1,
                                 delta_t / 2.0 * ang_v[0],
                                 delta_t / 2.0 * ang_v[1],
                                 delta_t / 2.0 * ang_v[2]);
      Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

      deskewed->at(i) = cloud->at(i);
      deskewed->at(i).getVector3fMap() = pt_;
    }

    return deskewed;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_queue;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;
};

}  // namespace s_graphs

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<s_graphs::PrefilteringNode>());
  rclcpp::shutdown();
  return 0;
}
