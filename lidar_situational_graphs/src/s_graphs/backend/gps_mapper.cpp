#include <s_graphs/backend/gps_mapper.hpp>

namespace s_graphs {

GPSMapper::GPSMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
  gps_edge_stddev_xy =
      node->get_parameter("gps_edge_stddev_xy").get_parameter_value().get<double>();
  gps_edge_stddev_z =
      node->get_parameter("gps_edge_stddev_z").get_parameter_value().get<double>();
}

GPSMapper::~GPSMapper() {}

bool GPSMapper::map_gps_data(
    std::shared_ptr<GraphSLAM>& covisibility_graph,
    std::deque<geographic_msgs::msg::GeoPointStamped::SharedPtr>& gps_queue,
    const std::map<int, KeyFrame::Ptr>& keyframes) {
  bool updated = false;
  auto gps_cursor = gps_queue.begin();

  for (auto& keyframe : keyframes) {
    if (keyframe.second->stamp > gps_queue.back()->header.stamp) {
      break;
    }

    if (keyframe.second->stamp < (*gps_cursor)->header.stamp ||
        keyframe.second->utm_coord) {
      continue;
    }

    // find the gps data which is closest to the keyframe
    auto closest_gps = gps_cursor;
    for (auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
      auto dt = (rclcpp::Time((*closest_gps)->header.stamp) - keyframe.second->stamp)
                    .seconds();
      auto dt2 =
          (rclcpp::Time((*gps)->header.stamp) - keyframe.second->stamp).seconds();
      if (std::abs(dt) < std::abs(dt2)) {
        break;
      }

      closest_gps = gps;
    }

    // if the time residual between the gps and keyframe is too large, skip it
    gps_cursor = closest_gps;
    if (0.2 <
        std::abs((rclcpp::Time((*closest_gps)->header.stamp) - keyframe.second->stamp)
                     .seconds())) {
      continue;
    }

    // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM
    // coordinate
    geodesy::UTMPoint utm;
    geodesy::fromMsg((*closest_gps)->position, utm);
    Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

    // the first gps data position will be the origin of the map
    if (!zero_utm) {
      zero_utm = xyz;
    }
    xyz -= (*zero_utm);

    keyframe.second->utm_coord = xyz;

    g2o::OptimizableGraph::Edge* edge;
    if (std::isnan(xyz.z())) {
      Eigen::Matrix2d information_matrix =
          Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
      shared_graph_mutex.lock();
      edge = covisibility_graph->add_se3_prior_xy_edge(
          keyframe.second->node, xyz.head<2>(), information_matrix);
      shared_graph_mutex.unlock();
    } else {
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
      information_matrix(2, 2) /= gps_edge_stddev_z;
      shared_graph_mutex.lock();
      edge = covisibility_graph->add_se3_prior_xyz_edge(
          keyframe.second->node, xyz, information_matrix);
      shared_graph_mutex.unlock();
    }

    shared_graph_mutex.lock();
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    shared_graph_mutex.unlock();

    updated = true;

    auto remove_loc = std::upper_bound(
        gps_queue.begin(),
        gps_queue.end(),
        keyframes.rbegin()->second->stamp,
        [=](const rclcpp::Time& stamp,
            const geographic_msgs::msg::GeoPointStamped::SharedPtr& geopoint) {
          return stamp < geopoint->header.stamp;
        });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }
  return updated;
}

}  // namespace s_graphs
