#include <rclcpp/logger.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>

namespace s_graphs {

GraphVisualizer::GraphVisualizer(const rclcpp::Node::SharedPtr node,
                                 std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
  node_ptr_ = node.get();

  map_frame_id =
      node->get_parameter("map_frame_id").get_parameter_value().get<std::string>();
  color_r = node->get_parameter("color_r").get_parameter_value().get<double>();
  color_g = node->get_parameter("color_g").get_parameter_value().get<double>();
  color_b = node->get_parameter("color_b").get_parameter_value().get<double>();

  keyframes_layer_id = "keyframes_layer";
  walls_layer_id = "walls_layer";
  rooms_layer_id = "rooms_layer";
  floors_layer_id = "floors_layer";

  std::string ns = node_ptr_->get_namespace();
  if (ns.length() > 1) {
    std::string ns_prefix = std::string(node_ptr_->get_namespace()).substr(1);
    map_frame_id = ns_prefix + "/" + map_frame_id;
  }

  keyframe_node_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      keyframes_layer_id, "/rviz_keyframe_node_visual_tools", node);
  keyframe_node_visual_tools->loadMarkerPub(false);
  keyframe_node_visual_tools->deleteAllMarkers();
  keyframe_node_visual_tools->enableBatchPublishing();
  keyframe_node_visual_tools->setAlpha(0.5);

  keyframe_edge_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      keyframes_layer_id, "/rviz_keyframe_edge_visual_tools", node);
  keyframe_edge_visual_tools->loadMarkerPub(false);
  keyframe_edge_visual_tools->deleteAllMarkers();
  keyframe_edge_visual_tools->enableBatchPublishing();
  keyframe_edge_visual_tools->setAlpha(1);

  plane_node_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      walls_layer_id, "/rviz_plane_node_visual_tools", node);
  plane_node_visual_tools->loadMarkerPub(false);
  plane_node_visual_tools->deleteAllMarkers();
  plane_node_visual_tools->enableBatchPublishing();
  plane_node_visual_tools->setAlpha(0.5);

  plane_edge_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      rooms_layer_id, "/rviz_plane_edge_visual_tools", node);
  plane_edge_visual_tools->loadMarkerPub(false);
  plane_edge_visual_tools->deleteAllMarkers();
  plane_edge_visual_tools->enableBatchPublishing();
  plane_edge_visual_tools->setAlpha(1);

  room_node_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      rooms_layer_id, "/rviz_room_node_visual_tools", node);
  room_node_visual_tools->loadMarkerPub(false);
  room_node_visual_tools->deleteAllMarkers();
  room_node_visual_tools->enableBatchPublishing();
  room_node_visual_tools->setAlpha(0.5);

  floor_edge_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      floors_layer_id, "/rviz_floor_edge_visual_tools", node);
  floor_edge_visual_tools->loadMarkerPub(false);
  floor_edge_visual_tools->deleteAllMarkers();
  floor_edge_visual_tools->enableBatchPublishing();
  floor_edge_visual_tools->setAlpha(1);

  floor_node_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      floors_layer_id, "/rviz_floor_node_visual_tools", node);
  floor_node_visual_tools->loadMarkerPub(false);
  floor_node_visual_tools->deleteAllMarkers();
  floor_node_visual_tools->enableBatchPublishing();
  floor_node_visual_tools->setAlpha(0.5);

  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

GraphVisualizer::~GraphVisualizer() {}

/**
 * @brief create visualization marker
 * @param stamp
 * @return
 */
visualization_msgs::msg::MarkerArray GraphVisualizer::visualize_covisibility_graph(
    const double loop_detector_radius,
    const rclcpp::Time& stamp,
    const g2o::SparseOptimizer* local_graph,
    std::vector<KeyFrame::Ptr> keyframes,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot,
    const std::unordered_map<int, InfiniteRooms> x_infinite_room_snapshot,
    const std::unordered_map<int, InfiniteRooms> y_infinite_room_snapshot,
    const std::unordered_map<int, Rooms> room_snapshot,
    const std::unordered_map<int, Floors> floors_vec) {
  visualization_msgs::msg::MarkerArray markers;

  // node markers
  double wall_vertex_h = 18;
  // lifetime
  rclcpp::Duration marker_lifetime = rclcpp::Duration::from_seconds(15);

  std::string keyframes_layer_id = "keyframes_layer";
  std::string walls_layer_id = "walls_layer";
  std::string rooms_layer_id = "rooms_layer";
  std::string floors_layer_id = "floors_layer";

  std::string ns = node_ptr_->get_namespace();
  if (ns.length() > 1) {
    std::string ns_prefix = std::string(node_ptr_->get_namespace()).substr(1);
    keyframes_layer_id = ns_prefix + "/" + keyframes_layer_id;
    walls_layer_id = ns_prefix + "/" + walls_layer_id;
    rooms_layer_id = ns_prefix + "/" + rooms_layer_id;
    floors_layer_id = ns_prefix + "/" + floors_layer_id;
  }

  visualization_msgs::msg::Marker traj_marker;
  traj_marker.header.frame_id = keyframes_layer_id;
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = markers.markers.size();
  traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.2;
  traj_marker.lifetime = marker_lifetime;
  visualization_msgs::msg::Marker imu_marker;
  imu_marker.header = traj_marker.header;
  imu_marker.ns = "imu";
  imu_marker.id = markers.markers.size() + 1;
  imu_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

  imu_marker.pose.orientation.w = 1.0;
  imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;
  imu_marker.lifetime = marker_lifetime;

  traj_marker.points.resize(keyframes.size());
  traj_marker.colors.resize(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    auto kf_map = local_graph->vertices().find(keyframes[i]->id());
    if (kf_map == local_graph->vertices().end()) continue;

    auto kf_node = dynamic_cast<g2o::VertexSE3*>(kf_map->second);
    Eigen::Vector3d pos = kf_node->estimate().translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();

    auto current_key_data = dynamic_cast<OptimizationData*>(kf_node->userData());

    auto current_floor = floors_vec.find(keyframes[i]->floor_level);

    if (current_key_data) {
      bool marginalized = false;
      current_key_data->get_marginalized_info(marginalized);
      bool stair_keyframe = false;
      current_key_data->get_stair_node_info(stair_keyframe);
      if (marginalized) {
        traj_marker.colors[i].r = 1.0;
        traj_marker.colors[i].g = 0.0;
        traj_marker.colors[i].b = 0.0;
        traj_marker.colors[i].a = 1.0;
      } else if (stair_keyframe) {
        traj_marker.colors[i].r = 0.0;
        traj_marker.colors[i].g = 0.0;
        traj_marker.colors[i].b = 1.0;
        traj_marker.colors[i].a = 1.0;
      } else {
        traj_marker.colors[i].r = current_floor->second.color[0] / 255;
        traj_marker.colors[i].g = current_floor->second.color[1] / 255;
        traj_marker.colors[i].b = current_floor->second.color[2] / 255;
        traj_marker.colors[i].a = 1.0;
      }
    } else if (kf_node->fixed()) {
      traj_marker.colors[i].r = 0.0;
      traj_marker.colors[i].g = 0.0;
      traj_marker.colors[i].b = 1.0;
      traj_marker.colors[i].a = 1.0;
    } else {
      traj_marker.colors[i].r = current_floor->second.color[0] / 255;
      traj_marker.colors[i].g = current_floor->second.color[1] / 255;
      traj_marker.colors[i].b = current_floor->second.color[2] / 255;
      traj_marker.colors[i].a = 1.0;
    }

    if (keyframes[i]->acceleration) {
      Eigen::Vector3d pos = kf_node->estimate().translation();
      geometry_msgs::msg::Point point;
      point.x = pos.x();
      point.y = pos.y();
      point.z = pos.z();

      std_msgs::msg::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.1;

      imu_marker.points.push_back(point);
      imu_marker.colors.push_back(color);
    }
  }
  markers.markers.push_back(traj_marker);
  markers.markers.push_back(imu_marker);

  // keyframe edge markers
  visualization_msgs::msg::Marker traj_edge_marker;
  traj_edge_marker.header.frame_id = keyframes_layer_id;
  traj_edge_marker.header.stamp = stamp;
  traj_edge_marker.ns = "keyframe_keyframe_edges";
  traj_edge_marker.id = markers.markers.size();
  traj_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  traj_edge_marker.lifetime = marker_lifetime;
  traj_edge_marker.pose.orientation.w = 1.0;
  traj_edge_marker.scale.x = 0.02;

  auto traj_edge_itr = local_graph->edges().begin();
  for (int i = 0; traj_edge_itr != local_graph->edges().end(); traj_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *traj_edge_itr;
    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (edge_se3) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);

      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2 = v2->estimate().translation();

      geometry_msgs::msg::Point point1, point2;
      point1.x = pt1.x();
      point1.y = pt1.y();
      point1.z = pt1.z();

      point2.x = pt2.x();
      point2.y = pt2.y();
      point2.z = pt2.z();
      traj_edge_marker.points.push_back(point1);
      traj_edge_marker.points.push_back(point2);

      double p1 = static_cast<double>(v1->id()) / local_graph->vertices().size();
      double p2 = static_cast<double>(v2->id()) / local_graph->vertices().size();

      std_msgs::msg::ColorRGBA color1, color2;
      color1.r = 0.0;
      color1.g = 0.0;
      color1.a = 1.0;

      color2.r = 0.0;
      color2.g = 0.0;
      color2.a = 1.0;
      traj_edge_marker.colors.push_back(color1);
      traj_edge_marker.colors.push_back(color2);
    }
  }
  markers.markers.push_back(traj_edge_marker);

  // keyframe plane edge markers
  visualization_msgs::msg::Marker traj_plane_edge_marker;
  traj_plane_edge_marker.header.frame_id = keyframes_layer_id;
  traj_plane_edge_marker.header.stamp = stamp;
  traj_plane_edge_marker.ns = "keyframe_plane_edges";
  traj_plane_edge_marker.id = markers.markers.size();
  traj_plane_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  traj_plane_edge_marker.lifetime = marker_lifetime;
  traj_plane_edge_marker.pose.orientation.w = 1.0;
  traj_plane_edge_marker.scale.x = 0.01;

  auto traj_plane_edge_itr = local_graph->edges().begin();
  for (int i = 0; traj_plane_edge_itr != local_graph->edges().end();
       traj_plane_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *traj_plane_edge_itr;
    g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);

    if (edge_plane) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_plane->vertices()[1]);

      if (!v1 || !v2) continue;

      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2;

      float r = 0, g = 0, b = 0.0;
      pcl::CentroidPoint<PointNormal> centroid;
      if (fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(1)) &&
          fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(2))) {
        pt2 = compute_plane_centroid<VerticalPlanes>(v2->id(), x_plane_snapshot);
        r = 0.0;
      } else if (fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(0)) &&
                 fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(2))) {
        pt2 = compute_plane_centroid<VerticalPlanes>(v2->id(), y_plane_snapshot);
        b = 0.0;
      } else if (fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(0)) &&
                 fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(1))) {
        pt2 = compute_plane_centroid<HorizontalPlanes>(v2->id(), hort_plane_snapshot);
        r = 0;
        g = 0.0;
      } else
        continue;

      if (pt2.x() == 0 && pt2.y() == 0 && pt2.z() == 0) continue;

      geometry_msgs::msg::Point point1, point2;
      geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
      point1.x = pt1.x();
      point1.y = pt1.y();
      point1.z = pt1.z();

      point2_stamped.header.frame_id = walls_layer_id;
      point2_stamped.point.x = pt2.x();
      point2_stamped.point.y = pt2.y();
      point2_stamped.point.z = pt2.z();
      tf_buffer->transform(point2_stamped,
                           point2_stamped_transformed,
                           keyframes_layer_id,
                           tf2::TimePointZero,
                           walls_layer_id);

      point2 = point2_stamped_transformed.point;
      traj_plane_edge_marker.points.push_back(point1);
      traj_plane_edge_marker.points.push_back(point2);

      std_msgs::msg::ColorRGBA color1, color2;
      color1.r = 0;
      color1.g = 0;
      color1.b = 0;
      color1.a = 1.0;

      color2.r = 0;
      color2.g = 0;
      color2.b = 0;
      color2.a = 1.0;
      traj_plane_edge_marker.colors.push_back(color1);
      traj_plane_edge_marker.colors.push_back(color2);
    }
  }
  markers.markers.push_back(traj_plane_edge_marker);

  // Wall edge markers
  // TODO:HB include this in the previous for loop
  visualization_msgs::msg::Marker wall_center_marker;
  auto wall_edge_itr = local_graph->edges().begin();
  for (int i = 0; wall_edge_itr != local_graph->edges().end(); wall_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *wall_edge_itr;
    g2o::EdgeWall2Planes* edge_wall = dynamic_cast<g2o::EdgeWall2Planes*>(edge);
    if (edge_wall) {
      g2o::VertexWallXYZ* v1 =
          dynamic_cast<g2o::VertexWallXYZ*>(edge_wall->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[1]);
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[2]);
      Eigen::Vector3d wall_center = v1->estimate();

      wall_center_marker.ns = "wall_center_marker";
      wall_center_marker.header.frame_id = map_frame_id;
      wall_center_marker.header.stamp = stamp;
      wall_center_marker.id = markers.markers.size() + 1;
      wall_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      wall_center_marker.lifetime = marker_lifetime;
      wall_center_marker.color.r = color_r;
      wall_center_marker.color.g = color_g;
      wall_center_marker.color.b = color_b;
      wall_center_marker.color.a = 1.0;
      wall_center_marker.scale.x = 0.3;
      wall_center_marker.scale.y = 0.3;
      wall_center_marker.scale.z = 0.3;
      wall_center_marker.pose.position.x = wall_center.x();
      wall_center_marker.pose.position.y = wall_center.y();
      wall_center_marker.pose.position.z = wall_vertex_h;
      wall_center_marker.pose.orientation.x = 0.0;
      wall_center_marker.pose.orientation.y = 0.0;
      wall_center_marker.pose.orientation.z = 0.0;
      wall_center_marker.pose.orientation.w = 1.0;
      markers.markers.push_back(wall_center_marker);
    }
  }

  // sphere
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = keyframes_layer_id;
  sphere_marker.header.stamp = stamp;
  sphere_marker.ns = "loop_close_radius";
  sphere_marker.id = markers.markers.size();
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.lifetime = marker_lifetime;
  if (!keyframes.empty()) {
    auto kf_map = local_graph->vertices().find(keyframes.back()->id());
    if (kf_map != local_graph->vertices().end()) {
      auto kf_node = dynamic_cast<g2o::VertexSE3*>(kf_map->second);
      Eigen::Vector3d pos = kf_node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z();
    }
  }
  sphere_marker.pose.orientation.w = 1.0;
  sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z =
      loop_detector_radius;
  sphere_marker.color.r = 1.0;
  sphere_marker.color.a = 0.3;
  markers.markers.push_back(sphere_marker);

  // x vertical plane markers
  visualization_msgs::msg::Marker x_vert_plane_marker =
      fill_plane_makers<VerticalPlanes>(x_plane_snapshot);
  x_vert_plane_marker.header.frame_id = walls_layer_id;
  x_vert_plane_marker.header.stamp = stamp;
  x_vert_plane_marker.ns = "x_vert_planes";
  x_vert_plane_marker.id = markers.markers.size();
  x_vert_plane_marker.lifetime = marker_lifetime;
  markers.markers.push_back(x_vert_plane_marker);

  // y vertical plane markers
  visualization_msgs::msg::Marker y_vert_plane_marker =
      fill_plane_makers<VerticalPlanes>(y_plane_snapshot);
  y_vert_plane_marker.header.frame_id = walls_layer_id;
  y_vert_plane_marker.header.stamp = stamp;
  y_vert_plane_marker.ns = "y_vert_planes";
  y_vert_plane_marker.id = markers.markers.size();
  y_vert_plane_marker.lifetime = marker_lifetime;
  markers.markers.push_back(y_vert_plane_marker);

  // horizontal plane markers
  visualization_msgs::msg::Marker hort_plane_marker =
      fill_plane_makers<HorizontalPlanes>(hort_plane_snapshot);
  hort_plane_marker.header.frame_id = walls_layer_id;
  hort_plane_marker.header.stamp = stamp;
  hort_plane_marker.ns = "hort_planes";
  hort_plane_marker.id = markers.markers.size();
  hort_plane_marker.lifetime = marker_lifetime;
  markers.markers.push_back(hort_plane_marker);

  // fil in the x_inf room marker
  this->fill_infinite_room(0,
                           stamp,
                           marker_lifetime,
                           local_graph,
                           x_plane_snapshot,
                           x_infinite_room_snapshot,
                           markers);

  // fill in the y_inf room marker
  this->fill_infinite_room(1,
                           stamp,
                           marker_lifetime,
                           local_graph,
                           y_plane_snapshot,
                           y_infinite_room_snapshot,
                           markers);

  // room markers
  for (const auto& room : room_snapshot) {
    auto room_map = local_graph->vertices().find(room.first);
    if (room_map == local_graph->vertices().end()) continue;

    auto floor_map = local_graph->vertices().find(room.second.floor_level);
    if (floor_map == local_graph->vertices().end()) {
      std::cout << "floor node not found for room" << std::endl;
      continue;
    }

    // fill in the line marker
    visualization_msgs::msg::Marker room_line_marker;
    room_line_marker.scale.x = 0.02;
    room_line_marker.pose.orientation.w = 1.0;
    room_line_marker.ns = "rooms_line";
    room_line_marker.header.frame_id = rooms_layer_id;
    room_line_marker.header.stamp = stamp;
    room_line_marker.id = markers.markers.size() + 1;
    room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    room_line_marker.lifetime = marker_lifetime;
    room_line_marker.color.r = color_r;
    room_line_marker.color.g = color_g;
    room_line_marker.color.b = color_b;
    room_line_marker.color.a = 1.0;
    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x =
        dynamic_cast<g2o::VertexRoom*>(room_map->second)->estimate().translation()(0);
    p1.y =
        dynamic_cast<g2o::VertexRoom*>(room_map->second)->estimate().translation()(1);
    p1.z =
        dynamic_cast<g2o::VertexFloor*>(floor_map->second)->estimate().translation()(2);

    auto found_planex1 = x_plane_snapshot.find(room.second.plane_x1_id);
    auto found_planex2 = x_plane_snapshot.find(room.second.plane_x2_id);
    auto found_planey1 = y_plane_snapshot.find(room.second.plane_y1_id);
    auto found_planey2 = y_plane_snapshot.find(room.second.plane_y2_id);

    p2 = compute_plane_point(p1, (*found_planex1).second.cloud_seg_map);
    if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p2);
    }

    p3 = compute_plane_point(p1, (*found_planex2).second.cloud_seg_map);
    if (p3.x == 0 && p3.y == 0 && p3.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p3);
    }

    p4 = compute_plane_point(p1, (*found_planey1).second.cloud_seg_map);
    if (p4.x = 0 && p4.y == 0 && p4.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p4);
    }

    p5 = compute_plane_point(p1, (*found_planey2).second.cloud_seg_map);
    if (p5.x == 0 && p5.y == 0 && p5.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p5);
    }
    markers.markers.push_back(room_line_marker);

    // fill the pose marker
    visualization_msgs::msg::Marker room_marker;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = rooms_layer_id;
    room_marker.header.stamp = stamp;
    room_marker.ns = "rooms";
    room_marker.id = markers.markers.size();
    room_marker.type = visualization_msgs::msg::Marker::CUBE;
    room_marker.color.r = 1;
    room_marker.color.g = 0.07;
    room_marker.color.b = 0.57;
    room_marker.color.a = 1;

    room_marker.pose.position.x = p1.x;
    room_marker.pose.position.y = p1.y;
    room_marker.pose.position.z = p1.z;

    Eigen::Quaterniond quat(
        dynamic_cast<g2o::VertexRoom*>(room_map->second)->estimate().linear());
    room_marker.pose.orientation.x = quat.x();
    room_marker.pose.orientation.y = quat.y();
    room_marker.pose.orientation.z = quat.z();
    room_marker.pose.orientation.w = quat.w();
    markers.markers.push_back(room_marker);
  }

  for (const auto& floor : floors_vec) {
    if (floor.first != -1) {
      auto floor_map = local_graph->vertices().find(floor.first);
      if (floor_map == local_graph->vertices().end()) {
        continue;
      }

      visualization_msgs::msg::Marker floor_marker;
      floor_marker.pose.orientation.w = 1.0;
      floor_marker.scale.x = 0.5;
      floor_marker.scale.y = 0.5;
      floor_marker.scale.z = 0.5;
      // plane_marker.points.resize(vert_planes.size());
      floor_marker.header.frame_id = floors_layer_id;
      floor_marker.header.stamp = stamp;
      floor_marker.ns = "floors";
      floor_marker.id = markers.markers.size();
      floor_marker.type = visualization_msgs::msg::Marker::CUBE;
      floor_marker.lifetime = marker_lifetime;
      floor_marker.color.r = 0.49;
      floor_marker.color.g = 0;
      floor_marker.color.b = 1;
      floor_marker.color.a = 1;

      floor_marker.pose.position.x = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                                         ->estimate()
                                         .translation()(0);
      floor_marker.pose.position.y = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                                         ->estimate()
                                         .translation()(1);
      floor_marker.pose.position.z = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                                         ->estimate()
                                         .translation()(2);

      // create line markers between floor and rooms/infinite_rooms
      visualization_msgs::msg::Marker floor_line_marker;
      floor_line_marker.scale.x = 0.02;
      floor_line_marker.pose.orientation.w = 1.0;
      floor_line_marker.ns = "floor_lines";
      floor_line_marker.header.frame_id = floors_layer_id;
      floor_line_marker.header.stamp = stamp;
      floor_line_marker.id = markers.markers.size() + 1;
      floor_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      floor_line_marker.lifetime = marker_lifetime;
      floor_line_marker.color.r = color_r;
      floor_line_marker.color.g = color_g;
      floor_line_marker.color.b = color_b;
      floor_line_marker.color.a = 1.0;

      for (const auto& room : room_snapshot) {
        if (room.second.floor_level != floor.first) continue;
        auto room_map = local_graph->vertices().find(room.first);
        if (room_map == local_graph->vertices().end()) continue;
        geometry_msgs::msg::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = dynamic_cast<g2o::VertexRoom*>(room_map->second)
                   ->estimate()
                   .translation()(0);
        p2.y = dynamic_cast<g2o::VertexRoom*>(room_map->second)
                   ->estimate()
                   .translation()(1);
        p2.z = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                   ->estimate()
                   .translation()(2);
        p2 = compute_room_point(p2);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& x_infinite_room : x_infinite_room_snapshot) {
        if (x_infinite_room.second.id == -1 ||
            x_infinite_room.second.floor_level != floor.first)
          continue;
        auto x_inf_room_map = local_graph->vertices().find(x_infinite_room.first);
        if (x_inf_room_map == local_graph->vertices().end()) continue;

        geometry_msgs::msg::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = dynamic_cast<g2o::VertexRoom*>(x_inf_room_map->second)
                   ->estimate()
                   .translation()(0);
        p2.y = dynamic_cast<g2o::VertexRoom*>(x_inf_room_map->second)
                   ->estimate()
                   .translation()(1);
        p2.z = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                   ->estimate()
                   .translation()(2);
        p2 = compute_room_point(p2);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& y_infinite_room : y_infinite_room_snapshot) {
        if (y_infinite_room.second.id == -1 ||
            y_infinite_room.second.floor_level != floor.first)
          continue;
        auto y_inf_room_map = local_graph->vertices().find(y_infinite_room.first);
        if (y_inf_room_map == local_graph->vertices().end()) continue;
        geometry_msgs::msg::Point p1, p2;
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = dynamic_cast<g2o::VertexRoom*>(y_inf_room_map->second)
                   ->estimate()
                   .translation()(0);
        p2.y = dynamic_cast<g2o::VertexRoom*>(y_inf_room_map->second)
                   ->estimate()
                   .translation()(1);
        p2.z = dynamic_cast<g2o::VertexFloor*>(floor_map->second)
                   ->estimate()
                   .translation()(2);
        p2 = compute_room_point(p2);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      markers.markers.push_back(floor_marker);
      markers.markers.push_back(floor_line_marker);
    }
  }

  return markers;
}

visualization_msgs::msg::MarkerArray GraphVisualizer::create_prior_marker_array(
    const rclcpp::Time& stamp,
    const g2o::SparseOptimizer* local_graph,
    std::vector<VerticalPlanes>& x_vert_planes_prior,
    std::vector<VerticalPlanes>& y_vert_planes_prior,
    std::vector<Rooms> rooms_vec_prior,
    std::vector<Rooms> rooms_vec,
    bool got_trans_prior2map_,
    const std::vector<DoorWays> doorways_vec_prior,
    std::vector<VerticalPlanes>& x_vert_planes,
    std::vector<VerticalPlanes>& y_vert_planes) {
  visualization_msgs::msg::MarkerArray prior_markers;
  double plane_h = 15;
  double wall_vertex_h = 18;
  double prior_room_h = 22;
  double deviation_h = 16;
  prior_markers.markers.clear();

  for (int i = 0; i < x_vert_planes_prior.size(); i++) {  // walls_x_coord.size()
    double r, g, b;
    visualization_msgs::msg::Marker wall_visual_marker;
    wall_visual_marker.header.frame_id = "prior_map";
    wall_visual_marker.header.stamp = stamp;
    wall_visual_marker.ns = "prior_x_walls";
    wall_visual_marker.id = prior_markers.markers.size() + i;
    wall_visual_marker.type = visualization_msgs::msg::Marker::CUBE;

    wall_visual_marker.pose.position.x = x_vert_planes_prior[i].start_point.x();
    wall_visual_marker.pose.position.y =
        x_vert_planes_prior[i].start_point.y() + 0.5 * x_vert_planes_prior[i].length;
    tf2::Quaternion rotation;
    rotation.setRPY(0, M_PI / 2, 0);  // 90 degrees in radians around the z-axis

    wall_visual_marker.pose.orientation.x = rotation.getX();
    wall_visual_marker.pose.orientation.y = rotation.getY();
    wall_visual_marker.pose.orientation.z = rotation.getZ();
    wall_visual_marker.pose.orientation.w = rotation.getW();

    wall_visual_marker.pose.position.z = plane_h;

    wall_visual_marker.scale.z = 0.01;
    wall_visual_marker.scale.x = 3.0;
    wall_visual_marker.scale.y = x_vert_planes_prior[i].length;

    wall_visual_marker.color.r = 0.7;  // x_vert_planes_prior[i].color[0] / 255;
    wall_visual_marker.color.g = 0.4;  // x_vert_planes_prior[i].color[1] / 255;
    wall_visual_marker.color.b = 0.2;  // x_vert_planes_prior[i].color[2] / 255;
    wall_visual_marker.color.a = 0.7;
    prior_markers.markers.push_back(wall_visual_marker);
  }

  for (int i = 0; i < y_vert_planes_prior.size(); i++) {  // walls_x_coord.size()

    double r, g, b;
    visualization_msgs::msg::Marker wall_visual_marker;
    wall_visual_marker.header.frame_id = "prior_map";
    wall_visual_marker.header.stamp = stamp;
    wall_visual_marker.ns = "prior_y_walls";
    wall_visual_marker.id = prior_markers.markers.size() + i;
    wall_visual_marker.type = visualization_msgs::msg::Marker::CUBE;

    wall_visual_marker.pose.position.x =
        y_vert_planes_prior[i].start_point.x() + 0.5 * y_vert_planes_prior[i].length;
    wall_visual_marker.pose.position.y = y_vert_planes_prior[i].start_point.y();
    wall_visual_marker.pose.position.z = plane_h;
    tf2::Quaternion rotation;
    rotation.setRPY(0, M_PI / 2, 0);  // 90 degrees in radians around the z-axis

    wall_visual_marker.pose.orientation.x = rotation.getX();
    wall_visual_marker.pose.orientation.y = rotation.getY();
    wall_visual_marker.pose.orientation.z = rotation.getZ();
    wall_visual_marker.pose.orientation.w = rotation.getW();

    wall_visual_marker.scale.y = 0.01;
    wall_visual_marker.scale.x = 3.0;
    wall_visual_marker.scale.z = y_vert_planes_prior[i].length;
    std_msgs::msg::ColorRGBA color;

    wall_visual_marker.color.r = 0.7;  // y_vert_planes_prior[i].color[0] / 255;
    wall_visual_marker.color.g = 0.4;  // y_vert_planes_prior[i].color[1] / 255;
    wall_visual_marker.color.b = 0.2;  // y_vert_planes_prior[i].color[2] / 255;
    wall_visual_marker.color.a = 0.2;
    prior_markers.markers.push_back(wall_visual_marker);
  }

  // Wall Center markers
  visualization_msgs::msg::Marker wall_center_marker;
  auto wall_edge_itr = local_graph->edges().begin();
  for (int i = 0; wall_edge_itr != local_graph->edges().end(); wall_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *wall_edge_itr;
    g2o::EdgeWall2Planes* edge_wall = dynamic_cast<g2o::EdgeWall2Planes*>(edge);
    if (edge_wall) {
      g2o::VertexWallXYZ* v1 =
          dynamic_cast<g2o::VertexWallXYZ*>(edge_wall->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[1]);
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[2]);
      Eigen::Vector3d wall_center = v1->estimate();

      wall_center_marker.ns = "wall_center_marker";
      wall_center_marker.header.frame_id = "prior_map";
      wall_center_marker.header.stamp = stamp;
      wall_center_marker.id = prior_markers.markers.size() + 1;
      wall_center_marker.type = visualization_msgs::msg::Marker::CUBE;

      wall_center_marker.color.r = 229 / 255.0;
      wall_center_marker.color.g = 170 / 255.0;
      wall_center_marker.color.b = 112 / 255.0;
      wall_center_marker.color.a = 1.0;
      wall_center_marker.scale.x = 0.3;
      wall_center_marker.scale.y = 0.3;
      wall_center_marker.scale.z = 0.3;
      wall_center_marker.pose.position.x = wall_center.x();
      wall_center_marker.pose.position.y = wall_center.y();
      wall_center_marker.pose.position.z = wall_vertex_h;
      wall_center_marker.pose.orientation.x = 0.0;
      wall_center_marker.pose.orientation.y = 0.0;
      wall_center_marker.pose.orientation.z = 0.0;
      wall_center_marker.pose.orientation.w = 1.0;
      // prior_markers.markers.push_back(wall_center_marker);

      // wall surface plane edge markers
      visualization_msgs::msg::Marker wall_edge_plane_marker;
      wall_edge_plane_marker.header.frame_id = "prior_map";
      wall_edge_plane_marker.header.stamp = stamp;
      wall_edge_plane_marker.ns = "wall_to_plane_edges";
      wall_edge_plane_marker.id = prior_markers.markers.size();
      wall_edge_plane_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      wall_edge_plane_marker.pose.orientation.w = 1.0;
      wall_edge_plane_marker.scale.x = 0.03;
      geometry_msgs::msg::Point point1, point2, point3;
      point1.x = wall_center.x();
      point1.y = wall_center.y();
      point1.z = wall_vertex_h;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (v2->id() == x_vert_planes_prior[j].id) {
          point2.x = x_vert_planes_prior[j].start_point.x();
          point2.y = x_vert_planes_prior[j].start_point.y();
          point2.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (v2->id() == y_vert_planes_prior[j].id) {
          point2.x = y_vert_planes_prior[j].start_point.x();
          point2.y = y_vert_planes_prior[j].start_point.y();
          point2.z = 1.0 + plane_h;
          break;
        }
      }
      wall_edge_plane_marker.points.push_back(point1);
      wall_edge_plane_marker.points.push_back(point2);
      wall_edge_plane_marker.color.r = 0.0;
      wall_edge_plane_marker.color.g = 0.0;
      wall_edge_plane_marker.color.b = 0.0;
      wall_edge_plane_marker.color.a = 1.0;
      // prior_markers.markers.push_back(wall_edge_plane_marker);
      visualization_msgs::msg::Marker wall_edge_plane_marker_2;
      wall_edge_plane_marker_2.header.frame_id = "prior_map";
      wall_edge_plane_marker_2.header.stamp = stamp;
      wall_edge_plane_marker_2.ns = "wall_to_plane_edges";
      wall_edge_plane_marker_2.id = prior_markers.markers.size();
      wall_edge_plane_marker_2.type = visualization_msgs::msg::Marker::LINE_LIST;
      wall_edge_plane_marker_2.pose.orientation.w = 1.0;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (v3->id() == x_vert_planes_prior[j].id) {
          point3.x = x_vert_planes_prior[j].start_point.x();
          point3.y = x_vert_planes_prior[j].start_point.y();
          point3.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (v3->id() == y_vert_planes_prior[j].id) {
          point3.x = y_vert_planes_prior[j].start_point.x();
          point3.y = y_vert_planes_prior[j].start_point.y();
          point3.z = 1.0 + plane_h;
          break;
        }
      }
      wall_edge_plane_marker_2.scale.x = 0.02;
      wall_edge_plane_marker_2.color.r = 0.0;
      wall_edge_plane_marker_2.color.g = 0.0;
      wall_edge_plane_marker_2.color.b = 0.0;
      wall_edge_plane_marker_2.color.a = 1.0;
      wall_edge_plane_marker_2.points.push_back(point1);
      wall_edge_plane_marker_2.points.push_back(point3);
      // prior_markers.markers.push_back(wall_edge_plane_marker_2);
    }
  }

  auto wall_dev_edge_iterator = local_graph->edges().begin();
  for (int i = 0; wall_dev_edge_iterator != local_graph->edges().end();
       wall_dev_edge_iterator++, i++) {
    g2o::HyperGraph::Edge* edge = *wall_dev_edge_iterator;
    g2o::EdgeSE3PlanePlane* edge_wall_dev = dynamic_cast<g2o::EdgeSE3PlanePlane*>(edge);
    if (edge_wall_dev) {
      g2o::VertexDeviation* v1 =
          dynamic_cast<g2o::VertexDeviation*>(edge_wall_dev->vertices()[0]);
      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(edge_wall_dev->vertices()[1]);
      g2o::VertexPlane* v3 =
          dynamic_cast<g2o::VertexPlane*>(edge_wall_dev->vertices()[2]);
      std::cout << "Deviation between : " << v2->id() << "  and: " << v3->id()
                << std::endl;
      int d = 0;
      if (abs(v2->estimate().coeffs()(0)) > abs(v2->estimate().coeffs()(1))) {
        Eigen::Vector4d a_graph_wall_coeffs = v2->estimate().toVector();
        pcl::PointXYZRGBNormal p_min, p_max;
        geometry_msgs::msg::Point p1, p2, p3;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        auto found_s_graph_plane =
            std::find_if(x_vert_planes.begin(),
                         x_vert_planes.end(),
                         boost::bind(&VerticalPlanes::id, _1) == v3->id());
        if (found_s_graph_plane != x_vert_planes.end()) {
          pose = compute_plane_pose(*found_s_graph_plane, p_min, p_max);

          Eigen::Matrix4d dev_pose = pose.matrix() * v1->estimate().matrix();
          visualization_msgs::msg::Marker deviation_marker;
          deviation_marker.header.frame_id = "prior_map";
          deviation_marker.header.stamp = stamp;
          deviation_marker.ns = "deviations";
          deviation_marker.id = prior_markers.markers.size() + i;
          deviation_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          deviation_marker.action = visualization_msgs::msg::Marker::ADD;
          deviation_marker.scale.x = 0.5;
          deviation_marker.scale.y = 0.5;
          deviation_marker.scale.z = 0.5;
          deviation_marker.color.r = 255.0;
          deviation_marker.color.g = 255.0;
          deviation_marker.color.b = 255.0;
          deviation_marker.color.a = 0.7;
          deviation_marker.text = "d";
          // std::cout << " A-graph plane : " << std::endl;
          // std::cout << v2->estimate().toVector() << std::endl;
          // std::cout << " S-graph plane : " << std::endl;
          // std::cout << v3->estimate().toVector() << std::endl;
          // std::cout << " dev pose  : " << std::endl;
          // std::cout << v1->estimate().matrix() << std::endl;

          Eigen::Vector3d translation = dev_pose.block<3, 1>(0, 3);
          Eigen::Matrix3d rotation_matrix = dev_pose.block<3, 3>(0, 0);
          Eigen::Quaterniond quaternion(rotation_matrix);
          quaternion.normalize();
          deviation_marker.pose.position.x = translation.x();
          p2.x = deviation_marker.pose.position.x;
          deviation_marker.pose.position.y = translation.y();
          p2.y = deviation_marker.pose.position.y;
          p2.z = plane_h;
          deviation_marker.pose.position.z = wall_vertex_h;
          deviation_marker.pose.orientation.x = quaternion.x();
          deviation_marker.pose.orientation.y = quaternion.y();
          deviation_marker.pose.orientation.z = quaternion.z();
          deviation_marker.pose.orientation.w = quaternion.w();

          prior_markers.markers.push_back(deviation_marker);
          Eigen::Isometry3d a_graph_wall_pose;
          auto found_a_graph_plane = x_vert_planes_prior.begin();
          found_a_graph_plane =
              std::find_if(x_vert_planes_prior.begin(),
                           x_vert_planes_prior.end(),
                           boost::bind(&s_graphs::VerticalPlanes::id, _1) == v2->id());

          a_graph_wall_pose.translation() = (*found_a_graph_plane).wall_point;
          a_graph_wall_pose.linear().setIdentity();
          double yaw =
              std::atan2(v2->estimate().coeffs()(1), v2->estimate().coeffs()(0));

          double pitch = std::atan2(v2->estimate().coeffs()(2),
                                    v2->estimate().coeffs().head<2>().norm());

          double roll = 0.0;
          Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

          a_graph_wall_pose.linear() = q.toRotationMatrix();

          p1.x = translation.x();
          p1.y = translation.y();
          p1.z = wall_vertex_h;
          p3.x = (*found_a_graph_plane).wall_point.x();
          p3.y = (*found_a_graph_plane).wall_point.y();
          p3.z = plane_h;

          visualization_msgs::msg::Marker deviation_wall_edge_marker;
          deviation_wall_edge_marker.header.frame_id = "prior_map";
          deviation_wall_edge_marker.header.stamp = stamp;
          deviation_wall_edge_marker.ns = "deviation_to_plane_edges";
          deviation_wall_edge_marker.id = prior_markers.markers.size();
          deviation_wall_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
          deviation_wall_edge_marker.pose.orientation.w = 1.0;
          deviation_wall_edge_marker.scale.x = 0.03;
          deviation_wall_edge_marker.scale.x = 0.02;
          deviation_wall_edge_marker.color.r = 0.0;
          deviation_wall_edge_marker.color.g = 0.0;
          deviation_wall_edge_marker.color.b = 0.0;
          deviation_wall_edge_marker.color.a = 1.0;
          deviation_wall_edge_marker.points.push_back(p1);
          deviation_wall_edge_marker.points.push_back(p3);

          prior_markers.markers.push_back(deviation_wall_edge_marker);

          Eigen::Vector3d text_marker_pose;
          text_marker_pose.x() = (p_min.x - p_max.x) / 2.0 + p_max.x;
          text_marker_pose.y() = (p_min.y - p_max.y) / 2.0 + p_max.y;
          text_marker_pose.z() = (p_min.z - p_max.z) / 2.0 + p_max.z;
          visualization_msgs::msg::Marker text_marker;
          text_marker.header.frame_id = "prior_map";  // Set the appropriate frame ID
          text_marker.header.stamp = stamp;
          text_marker.ns = "deviation_display";
          text_marker.id = prior_markers.markers.size();
          text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

          text_marker.pose.position.x = text_marker_pose.x();
          text_marker.pose.position.y = text_marker_pose.y();
          text_marker.pose.position.z = wall_vertex_h - 1;
          text_marker.pose.orientation.x = 0.0;
          text_marker.pose.orientation.y = 0.0;
          text_marker.pose.orientation.z = 0.0;
          text_marker.pose.orientation.w = 1.0;

          text_marker.scale.z = 0.1;

          text_marker.color.r = 255.0;
          text_marker.color.g = 255.0;
          text_marker.color.b = 255.0;
          text_marker.color.a = 1.0;

          Eigen::Vector3d dev_value = v1->estimate().matrix().block<3, 1>(0, 3);
          Eigen::Vector3d truncated_values;

          truncated_values[0] = std::floor(dev_value[0] * 100);
          truncated_values[1] = std::floor(dev_value[1] * 100);
          text_marker.text = std::to_string(truncated_values[0]) + "," +
                             std::to_string(truncated_values[1]);
          prior_markers.markers.push_back(text_marker);

          visualization_msgs::msg::Marker deviation_wall_edge_marker2;
          deviation_wall_edge_marker2.header.frame_id = "prior_map";
          deviation_wall_edge_marker2.header.stamp = stamp;
          deviation_wall_edge_marker2.ns = "deviation_to_plane_edges";
          deviation_wall_edge_marker2.id = prior_markers.markers.size();
          deviation_wall_edge_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
          deviation_wall_edge_marker2.pose.orientation.w = 1.0;
          deviation_wall_edge_marker2.scale.x = 0.03;
          deviation_wall_edge_marker2.scale.x = 0.02;
          deviation_wall_edge_marker2.color.r = 0.0;
          deviation_wall_edge_marker2.color.g = 0.0;
          deviation_wall_edge_marker2.color.b = 0.0;
          deviation_wall_edge_marker2.color.a = 1.0;
          deviation_wall_edge_marker2.points.push_back(p1);
          deviation_wall_edge_marker2.points.push_back(p2);
          prior_markers.markers.push_back(deviation_wall_edge_marker2);
          d++;
        }
      } else if (abs(v2->estimate().coeffs()(1)) > abs(v2->estimate().coeffs()(0))) {
        Eigen::Vector4d a_graph_wall_coeffs = v2->estimate().toVector();

        pcl::PointXYZRGBNormal p_min, p_max;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        geometry_msgs::msg::Point p1, p2, p3;
        auto found_s_graph_plane =
            std::find_if(y_vert_planes.begin(),
                         y_vert_planes.end(),
                         boost::bind(&VerticalPlanes::id, _1) == v3->id());
        if (found_s_graph_plane != y_vert_planes.end()) {
          float min_dist_x1 = 100;
          pose = compute_plane_pose(*found_s_graph_plane, p_min, p_max);

          Eigen::Matrix4d dev_pose = pose.matrix() * v1->estimate().matrix();
          visualization_msgs::msg::Marker deviation_marker;
          deviation_marker.header.frame_id = "prior_map";
          deviation_marker.header.stamp = stamp;
          deviation_marker.ns = "deviations";
          deviation_marker.id = prior_markers.markers.size() + i;
          deviation_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          deviation_marker.action = visualization_msgs::msg::Marker::ADD;
          deviation_marker.scale.x = 0.5;
          deviation_marker.scale.y = 0.5;
          deviation_marker.scale.z = 0.5;
          deviation_marker.color.r = 255.0;
          deviation_marker.color.g = 255.0;
          deviation_marker.color.b = 255.0;
          deviation_marker.color.a = 0.7;
          deviation_marker.text = "d";
          Eigen::Vector3d translation = dev_pose.block<3, 1>(0, 3);
          Eigen::Matrix3d rotation_matrix = dev_pose.block<3, 3>(0, 0);
          Eigen::Quaterniond quaternion(rotation_matrix);
          quaternion.normalize();
          deviation_marker.pose.position.x = translation.x();
          p2.x = deviation_marker.pose.position.x;
          deviation_marker.pose.position.y = translation.y();
          p2.y = deviation_marker.pose.position.y;
          deviation_marker.pose.position.z = wall_vertex_h;
          p2.z = plane_h;
          deviation_marker.pose.orientation.x = quaternion.x();
          deviation_marker.pose.orientation.y = quaternion.y();
          deviation_marker.pose.orientation.z = quaternion.z();
          deviation_marker.pose.orientation.w = quaternion.w();
          prior_markers.markers.push_back(deviation_marker);

          Eigen::Isometry3d a_graph_wall_pose = Eigen::Isometry3d::Identity();
          auto found_a_graph_plane = y_vert_planes_prior.begin();
          found_a_graph_plane =
              std::find_if(y_vert_planes_prior.begin(),
                           y_vert_planes_prior.end(),
                           boost::bind(&s_graphs::VerticalPlanes::id, _1) == v2->id());

          a_graph_wall_pose.translation() = (*found_a_graph_plane).wall_point;

          a_graph_wall_pose.linear().setIdentity();
          double yaw =
              std::atan2(v2->estimate().coeffs()(1), v2->estimate().coeffs()(0));

          double pitch = std::atan2(v2->estimate().coeffs()(2),
                                    v2->estimate().coeffs().head<2>().norm());

          double roll = 0.0;
          Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

          a_graph_wall_pose.linear() = q.toRotationMatrix();

          p1.x = translation.x();
          p1.y = translation.y();
          p1.z = wall_vertex_h;
          p3.x = (*found_a_graph_plane).wall_point.x();
          p3.y = (*found_a_graph_plane).wall_point.y();
          p3.z = plane_h;

          visualization_msgs::msg::Marker deviation_wall_edge_marker;
          deviation_wall_edge_marker.header.frame_id = "prior_map";
          deviation_wall_edge_marker.header.stamp = stamp;
          deviation_wall_edge_marker.ns = "deviation_to_plane_edges";
          deviation_wall_edge_marker.id = prior_markers.markers.size();
          deviation_wall_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
          deviation_wall_edge_marker.pose.orientation.w = 1.0;
          deviation_wall_edge_marker.scale.x = 0.03;
          deviation_wall_edge_marker.scale.x = 0.02;
          deviation_wall_edge_marker.color.r = 0.0;
          deviation_wall_edge_marker.color.g = 0.0;
          deviation_wall_edge_marker.color.b = 0.0;
          deviation_wall_edge_marker.color.a = 1.0;
          deviation_wall_edge_marker.points.push_back(p1);
          deviation_wall_edge_marker.points.push_back(p3);

          prior_markers.markers.push_back(deviation_wall_edge_marker);

          Eigen::Vector3d text_marker_pose;
          text_marker_pose.x() = (p_min.x - p_max.x) / 2.0 + p_max.x;
          text_marker_pose.y() = (p_min.y - p_max.y) / 2.0 + p_max.y;
          text_marker_pose.z() = (p_min.z - p_max.z) / 2.0 + p_max.z;

          visualization_msgs::msg::Marker text_marker;
          text_marker.header.frame_id = "prior_map";  // Set the appropriate frame ID
          text_marker.header.stamp = stamp;
          text_marker.ns = "deviation_display";
          text_marker.id = prior_markers.markers.size();
          text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

          text_marker.pose.position.x = text_marker_pose.x();
          text_marker.pose.position.y = text_marker_pose.y();
          text_marker.pose.position.z = wall_vertex_h - 1;
          text_marker.pose.orientation.x = 0.0;
          text_marker.pose.orientation.y = 0.0;
          text_marker.pose.orientation.z = 0.0;
          text_marker.pose.orientation.w = 1.0;

          text_marker.scale.z = 0.1;

          text_marker.color.r = 255.0;
          text_marker.color.g = 255.0;
          text_marker.color.b = 255.0;
          text_marker.color.a = 1.0;

          Eigen::Vector3d dev_value = v1->estimate().matrix().block<3, 1>(0, 3);
          Eigen::Vector3d truncated_values;
          truncated_values[0] = std::floor(dev_value[0] * 100);
          truncated_values[1] = std::floor(dev_value[1] * 100);
          text_marker.text = std::to_string(truncated_values[0]) + "," +
                             std::to_string(truncated_values[1]);
          prior_markers.markers.push_back(text_marker);

          visualization_msgs::msg::Marker deviation_wall_edge_marker2;
          deviation_wall_edge_marker2.header.frame_id = "prior_map";
          deviation_wall_edge_marker2.header.stamp = stamp;
          deviation_wall_edge_marker2.ns = "deviation_to_plane_edges";
          deviation_wall_edge_marker2.id = prior_markers.markers.size();
          deviation_wall_edge_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
          deviation_wall_edge_marker2.pose.orientation.w = 1.0;
          deviation_wall_edge_marker2.scale.x = 0.03;
          deviation_wall_edge_marker2.scale.x = 0.02;
          deviation_wall_edge_marker2.color.r = 0.0;
          deviation_wall_edge_marker2.color.g = 0.0;
          deviation_wall_edge_marker2.color.b = 0.0;
          deviation_wall_edge_marker2.color.a = 1.0;
          deviation_wall_edge_marker2.points.push_back(p1);
          deviation_wall_edge_marker2.points.push_back(p2);
          prior_markers.markers.push_back(deviation_wall_edge_marker2);
          d++;
        }
      }
    }
  }
  if (!got_trans_prior2map_) {
    for (int i = 0; i < rooms_vec_prior.size(); i++) {  // walls_x_coord.size()
      double r, g, b;
      visualization_msgs::msg::Marker prior_room_marker;
      prior_room_marker.header.frame_id = "prior_map";
      prior_room_marker.header.stamp = stamp;
      prior_room_marker.ns = "prior_rooms";
      prior_room_marker.id = prior_markers.markers.size() + i;
      prior_room_marker.type = visualization_msgs::msg::Marker::CUBE;
      Eigen::Matrix4d room_pose = rooms_vec_prior[i].node->estimate().matrix();
      prior_room_marker.pose.position.x = room_pose(0, 3);
      prior_room_marker.pose.position.y = room_pose(1, 3);
      prior_room_marker.pose.position.z = prior_room_h;
      prior_room_marker.pose.orientation.x = 0;
      prior_room_marker.pose.orientation.y = 0;
      prior_room_marker.pose.orientation.z = 0.0;
      prior_room_marker.pose.orientation.w = 1;

      prior_room_marker.scale.y = 0.2;
      prior_room_marker.scale.x = 0.2;
      prior_room_marker.scale.z = 0.2;
      prior_room_marker.color.r = 0.0;
      prior_room_marker.color.g = 0.7;
      prior_room_marker.color.b = 1.0;
      prior_room_marker.color.a = 0.5;
      prior_markers.markers.push_back(prior_room_marker);

      // Edge plane 1
      geometry_msgs::msg::Point point1, point2, point3, point4, point5;
      point1.x = room_pose(0, 3);
      point1.y = room_pose(1, 3);
      point1.z = prior_room_h;
      visualization_msgs::msg::Marker room_edge_plane_marker1;
      room_edge_plane_marker1.header.frame_id = "prior_map";
      room_edge_plane_marker1.header.stamp = stamp;
      room_edge_plane_marker1.ns = "prior_room_to_plane_markers";
      room_edge_plane_marker1.id = prior_markers.markers.size();
      room_edge_plane_marker1.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_edge_plane_marker1.pose.orientation.w = 1.0;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_x1_id == x_vert_planes_prior[j].id) {
          point2.x = x_vert_planes_prior[j].start_point.x();
          point2.y = room_pose(1, 3);  // x_vert_planes_prior[j].start_point.y();
          point2.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_x1_id == y_vert_planes_prior[j].id) {
          point2.x = room_pose(0, 3);  // y_vert_planes_prior[j].start_point.x();
          point2.y = y_vert_planes_prior[j].start_point.y();
          point2.z = 1.0 + plane_h;
          break;
        }
      }
      room_edge_plane_marker1.scale.x = 0.01;
      room_edge_plane_marker1.color.r = 0.0;
      room_edge_plane_marker1.color.g = 0.0;
      room_edge_plane_marker1.color.b = 0.0;
      room_edge_plane_marker1.color.a = 0.5;
      room_edge_plane_marker1.points.push_back(point1);
      room_edge_plane_marker1.points.push_back(point2);
      prior_markers.markers.push_back(room_edge_plane_marker1);

      visualization_msgs::msg::Marker room_edge_plane_marker2;
      room_edge_plane_marker2.header.frame_id = "prior_map";
      room_edge_plane_marker2.header.stamp = stamp;
      room_edge_plane_marker2.ns = "prior_room_to_plane_markers";
      room_edge_plane_marker2.id = prior_markers.markers.size();
      room_edge_plane_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_edge_plane_marker2.pose.orientation.w = 1.0;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_x2_id == x_vert_planes_prior[j].id) {
          point3.x = x_vert_planes_prior[j].start_point.x();
          point3.y = room_pose(1, 3);  // x_vert_planes_prior[j].start_point.y();
          point3.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_x2_id == y_vert_planes_prior[j].id) {
          point3.x = room_pose(0, 3);  // y_vert_planes_prior[j].start_point.x();
          point3.y = y_vert_planes_prior[j].start_point.y();
          point3.z = 1.0 + plane_h;
          break;
        }
      }
      room_edge_plane_marker2.scale.x = 0.01;
      room_edge_plane_marker2.color.r = 0.0;
      room_edge_plane_marker2.color.g = 0.0;
      room_edge_plane_marker2.color.b = 0.0;
      room_edge_plane_marker2.color.a = 0.5;
      room_edge_plane_marker2.points.push_back(point1);
      room_edge_plane_marker2.points.push_back(point3);
      prior_markers.markers.push_back(room_edge_plane_marker2);

      visualization_msgs::msg::Marker room_edge_plane_marker3;
      room_edge_plane_marker3.header.frame_id = "prior_map";
      room_edge_plane_marker3.header.stamp = stamp;
      room_edge_plane_marker3.ns = "prior_room_to_plane_markers";
      room_edge_plane_marker3.id = prior_markers.markers.size();
      room_edge_plane_marker3.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_edge_plane_marker3.pose.orientation.w = 1.0;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_y1_id == x_vert_planes_prior[j].id) {
          point4.x = x_vert_planes_prior[j].start_point.x();
          point4.y = room_pose(1, 3);  // x_vert_planes_prior[j].start_point.y();
          point4.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_y1_id == y_vert_planes_prior[j].id) {
          point4.x = room_pose(0, 3);  // y_vert_planes_prior[j].start_point.x();
          point4.y = y_vert_planes_prior[j].start_point.y();
          point4.z = 1.0 + plane_h;
          break;
        }
      }
      room_edge_plane_marker3.scale.x = 0.01;
      room_edge_plane_marker3.color.r = 0.0;
      room_edge_plane_marker3.color.g = 0.0;
      room_edge_plane_marker3.color.b = 0.0;
      room_edge_plane_marker3.color.a = 0.5;
      room_edge_plane_marker3.points.push_back(point1);
      room_edge_plane_marker3.points.push_back(point4);
      prior_markers.markers.push_back(room_edge_plane_marker3);

      visualization_msgs::msg::Marker room_edge_plane_marker4;
      room_edge_plane_marker4.header.frame_id = "prior_map";
      room_edge_plane_marker4.header.stamp = stamp;
      room_edge_plane_marker4.ns = "prior_room_to_plane_markers";
      room_edge_plane_marker4.id = prior_markers.markers.size();
      room_edge_plane_marker4.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_edge_plane_marker4.pose.orientation.w = 1.0;
      for (int j = 0; j < x_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_y2_id == x_vert_planes_prior[j].id) {
          point5.x = x_vert_planes_prior[j].start_point.x();
          point5.y = room_pose(1, 3);  // x_vert_planes_prior[j].start_point.y();
          point5.z = 1.0 + plane_h;
          break;
        }
      }
      for (int j = 0; j < y_vert_planes_prior.size(); j++) {
        if (rooms_vec_prior[i].plane_y2_id == y_vert_planes_prior[j].id) {
          point5.x = room_pose(0, 3);  // y_vert_planes_prior[j].start_point.x();
          point5.y = y_vert_planes_prior[j].start_point.y();
          point5.z = 1.0 + plane_h;
          break;
        }
      }
      room_edge_plane_marker4.scale.x = 0.01;
      room_edge_plane_marker4.color.r = 0.0;
      room_edge_plane_marker4.color.g = 0.0;
      room_edge_plane_marker4.color.b = 0.0;
      room_edge_plane_marker4.color.a = 0.5;
      room_edge_plane_marker4.points.push_back(point1);
      room_edge_plane_marker4.points.push_back(point5);
      prior_markers.markers.push_back(room_edge_plane_marker4);
    }
  }
  for (int i = 0; i < doorways_vec_prior.size(); i++) {  // walls_x_coord.size()

    double r, g, b;
    visualization_msgs::msg::Marker prior_door_marker;
    prior_door_marker.header.frame_id = "prior_map";
    prior_door_marker.header.stamp = stamp;
    prior_door_marker.ns = "prior_doorways";
    prior_door_marker.id = prior_markers.markers.size() + i;
    prior_door_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    prior_door_marker.mesh_resource = "package://is_graphs/meshes/door.dae";
    Eigen::Vector3d door_pose = doorways_vec_prior[i].door_pos_w;
    prior_door_marker.pose.position.x = door_pose.x();
    prior_door_marker.pose.position.y = door_pose.y();
    prior_door_marker.pose.position.z = prior_room_h;
    prior_door_marker.pose.orientation.x = 0;
    prior_door_marker.pose.orientation.y = 0;
    prior_door_marker.pose.orientation.z = 0.0;
    prior_door_marker.pose.orientation.w = 1;

    prior_door_marker.scale.y = 0.08;
    prior_door_marker.scale.x = 0.5;
    prior_door_marker.scale.z = 0.5;
    prior_door_marker.color.r = 193 / 255.0;
    prior_door_marker.color.g = 154 / 255.0;
    prior_door_marker.color.b = 107 / 255.0;
    prior_door_marker.color.a = 1.0;
    prior_markers.markers.push_back(prior_door_marker);
    if (!got_trans_prior2map_) {
      geometry_msgs::msg::Point point1, point2, point3;
      point1.x = door_pose.x();
      point1.y = door_pose.y();
      point1.z = prior_room_h;
      visualization_msgs::msg::Marker door_edge_plane_marker1;
      door_edge_plane_marker1.header.frame_id = "prior_map";
      door_edge_plane_marker1.header.stamp = stamp;
      door_edge_plane_marker1.ns = "prior_door_edges";
      door_edge_plane_marker1.id = prior_markers.markers.size();
      door_edge_plane_marker1.type = visualization_msgs::msg::Marker::LINE_LIST;
      door_edge_plane_marker1.pose.orientation.w = 1.0;
      for (int j = 0; j < rooms_vec_prior.size(); j++) {
        if (rooms_vec_prior[j].prior_id == doorways_vec_prior[i].room1_id) {
          Eigen::Matrix4d room_pose = rooms_vec_prior[j].node->estimate().matrix();
          point2.x = room_pose(0, 3);
          point2.y = room_pose(1, 3);
          point2.z = prior_room_h;
        }
      }

      door_edge_plane_marker1.scale.x = 0.01;
      door_edge_plane_marker1.color.r = 0.0;
      door_edge_plane_marker1.color.g = 0.0;
      door_edge_plane_marker1.color.b = 0.0;
      door_edge_plane_marker1.color.a = 0.5;
      door_edge_plane_marker1.points.push_back(point1);
      door_edge_plane_marker1.points.push_back(point2);
      prior_markers.markers.push_back(door_edge_plane_marker1);

      visualization_msgs::msg::Marker door_edge_plane_marker2;
      door_edge_plane_marker2.header.frame_id = "prior_map";
      door_edge_plane_marker2.header.stamp = stamp;
      door_edge_plane_marker2.ns = "prior_door_edges";
      door_edge_plane_marker2.id = prior_markers.markers.size();
      door_edge_plane_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
      door_edge_plane_marker2.pose.orientation.w = 1.0;
      for (int j = 0; j < rooms_vec_prior.size(); j++) {
        if (rooms_vec_prior[j].prior_id == doorways_vec_prior[i].room2_id) {
          Eigen::Matrix4d room_pose = rooms_vec_prior[j].node->estimate().matrix();
          point3.x = room_pose(0, 3);
          point3.y = room_pose(1, 3);
          point3.z = prior_room_h;
        }
      }
      door_edge_plane_marker2.scale.x = 0.01;
      door_edge_plane_marker2.color.r = 0.0;
      door_edge_plane_marker2.color.g = 0.0;
      door_edge_plane_marker2.color.b = 0.0;
      door_edge_plane_marker2.color.a = 0.5;
      door_edge_plane_marker2.points.push_back(point1);
      door_edge_plane_marker2.points.push_back(point3);
      prior_markers.markers.push_back(door_edge_plane_marker2);
    }

    if (got_trans_prior2map_) {
      geometry_msgs::msg::Point point1, point2, point3;
      point1.x = door_pose.x();
      point1.y = door_pose.y();
      point1.z = prior_room_h;
      visualization_msgs::msg::Marker door_edge_plane_marker1;
      door_edge_plane_marker1.header.frame_id = "prior_map";
      door_edge_plane_marker1.header.stamp = stamp;
      door_edge_plane_marker1.ns = "prior_door_edges";
      door_edge_plane_marker1.id = prior_markers.markers.size();
      door_edge_plane_marker1.type = visualization_msgs::msg::Marker::LINE_LIST;
      door_edge_plane_marker1.pose.orientation.w = 1.0;
      for (int j = 0; j < rooms_vec.size(); j++) {
        if (rooms_vec[j].prior_id == doorways_vec_prior[i].room1_id) {
          Eigen::Matrix4d room_pose = rooms_vec[j].node->estimate().matrix();
          point2.x = room_pose(0, 3);
          point2.y = room_pose(1, 3);
          point2.z = prior_room_h;
        }
      }

      door_edge_plane_marker1.scale.x = 0.02;
      door_edge_plane_marker1.color.r = 0.0;
      door_edge_plane_marker1.color.g = 0.0;
      door_edge_plane_marker1.color.b = 0.0;
      door_edge_plane_marker1.color.a = 0.5;
      door_edge_plane_marker1.points.push_back(point1);
      door_edge_plane_marker1.points.push_back(point2);
      prior_markers.markers.push_back(door_edge_plane_marker1);

      visualization_msgs::msg::Marker door_edge_plane_marker2;
      door_edge_plane_marker2.header.frame_id = "prior_map";
      door_edge_plane_marker2.header.stamp = stamp;
      door_edge_plane_marker2.ns = "prior_door_edges";
      door_edge_plane_marker2.id = prior_markers.markers.size();
      door_edge_plane_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
      door_edge_plane_marker2.pose.orientation.w = 1.0;
      for (int j = 0; j < rooms_vec.size(); j++) {
        if (rooms_vec[j].prior_id == doorways_vec_prior[i].room2_id) {
          Eigen::Matrix4d room_pose = rooms_vec[j].node->estimate().matrix();
          point3.x = room_pose(0, 3);
          point3.y = room_pose(1, 3);
          point3.z = prior_room_h;
        }
      }
      door_edge_plane_marker2.scale.x = 0.02;
      door_edge_plane_marker2.color.r = 0.0;
      door_edge_plane_marker2.color.g = 0.0;
      door_edge_plane_marker2.color.b = 0.0;
      door_edge_plane_marker2.color.a = 0.5;
      door_edge_plane_marker2.points.push_back(point1);
      door_edge_plane_marker2.points.push_back(point3);
      prior_markers.markers.push_back(door_edge_plane_marker2);
    }
  }
  return prior_markers;
}

void GraphVisualizer::visualize_compressed_graph(
    const rclcpp::Time& stamp,
    bool global_optimization,
    bool room_optimization,
    const g2o::SparseOptimizer* compressed_graph,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot) {
  keyframe_node_visual_tools->deleteAllMarkers();
  rviz_visual_tools::Colors node_color;
  rviz_visual_tools::Colors keyframe_edge_color, keyframe_plane_edge_color;
  rviz_visual_tools::Colors keyframe_node_color;

  if (global_optimization) {
    keyframe_node_color = rviz_visual_tools::ORANGE;
  } else if (room_optimization) {
    keyframe_node_color = rviz_visual_tools::RED;
  } else {
    keyframe_node_color = rviz_visual_tools::TRANSLUCENT;
  }

  node_color = rviz_visual_tools::TRANSLUCENT;
  keyframe_edge_color = keyframe_plane_edge_color = rviz_visual_tools::TRANSLUCENT;

  for (const auto vertex : compressed_graph->vertices()) {
    const g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(vertex.second);
    if (vertex_se3) {
      Eigen::Isometry3d pose;
      double depth = 0.4, width = 0.4, height = 0.4;

      pose.translation() = Eigen::Vector3d(vertex_se3->estimate().translation()(0),
                                           vertex_se3->estimate().translation()(1),
                                           vertex_se3->estimate().translation()(2));
      pose.linear().setIdentity();

      if (!vertex_se3->fixed())
        keyframe_node_visual_tools->publishCuboid(
            pose, depth, width, height, keyframe_node_color);
      else {
        keyframe_node_visual_tools->publishCuboid(
            pose, depth, width, height, rviz_visual_tools::BLUE);
      }
    }
  }

  keyframe_node_visual_tools->trigger();

  return;
}

Eigen::Isometry3d GraphVisualizer::compute_plane_pose(const VerticalPlanes& plane,
                                                      pcl::PointXYZRGBNormal& p_min,
                                                      pcl::PointXYZRGBNormal& p_max) {
  shared_graph_mutex.lock();
  double length = pcl::getMaxSegment(*plane.cloud_seg_map, p_min, p_max);
  shared_graph_mutex.unlock();

  Eigen::Isometry3d pose;
  pose.translation() = Eigen::Vector3d((p_min.x - p_max.x) / 2.0 + p_max.x,
                                       (p_min.y - p_max.y) / 2.0 + p_max.y,
                                       (p_min.z - p_max.z) / 2.0 + p_max.z);

  pose.linear().setIdentity();
  double yaw = std::atan2(plane.plane_node->estimate().coeffs()(1),
                          plane.plane_node->estimate().coeffs()(0));

  double pitch = std::atan2(plane.plane_node->estimate().coeffs()(2),
                            plane.plane_node->estimate().coeffs().head<2>().norm());

  double roll = 0.0;
  Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  pose.linear() = q.toRotationMatrix();
  return pose;
}

template <typename T>
visualization_msgs::msg::Marker GraphVisualizer::fill_plane_makers(
    const std::unordered_map<int, T>& plane_snapshot) {
  visualization_msgs::msg::Marker plane_marker;
  plane_marker.pose.orientation.w = 1.0;
  plane_marker.scale.x = 0.05;
  plane_marker.scale.y = 0.05;
  plane_marker.scale.z = 0.05;
  plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

  for (const auto& plane : plane_snapshot) {
    std_msgs::msg::ColorRGBA color;
    color.r = plane.second.color[0] / 255;
    color.g = plane.second.color[1] / 255;
    color.b = plane.second.color[2] / 255;
    color.a = 0.5;

    shared_graph_mutex.lock();
    if (plane.second.cloud_seg_map == nullptr) continue;
    auto local_cloud_seg_map =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
            *plane.second.cloud_seg_map);
    shared_graph_mutex.unlock();

    for (int j = 0; j < local_cloud_seg_map->points.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = local_cloud_seg_map->points[j].x;
      point.y = local_cloud_seg_map->points[j].y;
      point.z = local_cloud_seg_map->points[j].z;
      plane_marker.points.push_back(point);
      plane_marker.colors.push_back(color);
    }
  }

  return plane_marker;
}

template <typename T>
Eigen::Vector3d GraphVisualizer::compute_plane_centroid(
    const int current_plane_id,
    const std::unordered_map<int, T>& plane_snapshot) {
  Eigen::Vector3d pt(0, 0, 0);
  auto plane = plane_snapshot.find(current_plane_id);

  if (plane != plane_snapshot.end()) {
    shared_graph_mutex.lock();
    if (plane->second.cloud_seg_map->points.empty()) {
      shared_graph_mutex.unlock();
      return pt;
    }

    auto local_cloud_seg_map =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
            *plane->second.cloud_seg_map);
    shared_graph_mutex.unlock();

    double x = 0, y = 0, z = 0;
    for (int p = 0; p < local_cloud_seg_map->points.size(); ++p) {
      x += local_cloud_seg_map->points[p].x;
      y += local_cloud_seg_map->points[p].y;
      z += local_cloud_seg_map->points[p].z;
    }
    x = x / local_cloud_seg_map->points.size();
    y = y / local_cloud_seg_map->points.size();
    z = z / local_cloud_seg_map->points.size();
    pt = Eigen::Vector3d(x, y, z);
  }

  return pt;
}

geometry_msgs::msg::Point GraphVisualizer::compute_plane_point(
    geometry_msgs::msg::Point room_p1,
    const pcl::PointCloud<PointNormal>::Ptr cloud_seg_map) {
  float min_dist_plane1 = 100;
  geometry_msgs::msg::Point plane_p2;
  plane_p2.x = plane_p2.y = plane_p2.z = 0;

  shared_graph_mutex.lock();
  auto local_cloud_seg_map =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>(*cloud_seg_map);
  shared_graph_mutex.unlock();

  for (int p = 0; p < local_cloud_seg_map->points.size(); ++p) {
    geometry_msgs::msg::Point p_tmp;
    p_tmp.x = local_cloud_seg_map->points[p].x;
    p_tmp.y = local_cloud_seg_map->points[p].y;
    p_tmp.z = cloud_seg_map->points[p].z;

    float norm = std::sqrt(std::pow((room_p1.x - p_tmp.x), 2) +
                           std::pow((room_p1.y - p_tmp.y), 2) +
                           std::pow((room_p1.z - p_tmp.z), 2));

    if (norm < min_dist_plane1) {
      min_dist_plane1 = norm;
      plane_p2 = p_tmp;
    }
  }

  geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
  point2_stamped.header.frame_id = walls_layer_id;
  point2_stamped.point.x = plane_p2.x;
  point2_stamped.point.y = plane_p2.y;
  point2_stamped.point.z = plane_p2.z;

  // convert point p2 to rooms_layer_id currently it is map_frame_id
  tf_buffer->transform(point2_stamped,
                       point2_stamped_transformed,
                       rooms_layer_id,
                       tf2::TimePointZero,
                       walls_layer_id);

  return point2_stamped_transformed.point;
}

geometry_msgs::msg::Point GraphVisualizer::compute_room_point(
    geometry_msgs::msg::Point room_p1) {
  geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
  point2_stamped.header.frame_id = rooms_layer_id;
  point2_stamped.point.x = room_p1.x;
  point2_stamped.point.y = room_p1.y;
  point2_stamped.point.z = room_p1.z;

  // convert point p2 to rooms_layer_id currently it is map_frame_id
  tf_buffer->transform(point2_stamped,
                       point2_stamped_transformed,
                       floors_layer_id,
                       tf2::TimePointZero,
                       rooms_layer_id);
  room_p1 = point2_stamped_transformed.point;

  return room_p1;
}

void GraphVisualizer::fill_infinite_room(
    const int& room_class,
    const rclcpp::Time& stamp,
    rclcpp::Duration marker_lifetime,
    const g2o::SparseOptimizer* local_graph,
    const std::unordered_map<int, VerticalPlanes>& plane_snapshot,
    std::unordered_map<int, InfiniteRooms> infinite_room_snapshot,
    visualization_msgs::msg::MarkerArray& markers) {
  // fill in the line marker
  for (const auto& infinite_room : infinite_room_snapshot) {
    auto inf_room_map = local_graph->vertices().find(infinite_room.first);
    if (inf_room_map == local_graph->vertices().end()) continue;

    auto floor_map = local_graph->vertices().find(infinite_room.second.floor_level);
    if (floor_map == local_graph->vertices().end()) {
      std::cout << "floor node not found for inf room" << std::endl;
      continue;
    }

    visualization_msgs::msg::Marker infinite_room_line_marker;
    infinite_room_line_marker.scale.x = 0.02;
    infinite_room_line_marker.pose.orientation.w = 1.0;
    if (room_class == 0) infinite_room_line_marker.ns = "infinite_room_x_lines";
    if (room_class == 1) infinite_room_line_marker.ns = "infinite_room_y_lines";
    infinite_room_line_marker.header.frame_id = rooms_layer_id;
    infinite_room_line_marker.header.stamp = stamp;
    infinite_room_line_marker.id = markers.markers.size() + 1;
    infinite_room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    infinite_room_line_marker.lifetime = marker_lifetime;
    infinite_room_line_marker.color.r = color_r;
    infinite_room_line_marker.color.g = color_g;
    infinite_room_line_marker.color.b = color_b;
    infinite_room_line_marker.color.a = 1.0;

    auto found_plane1 = plane_snapshot.find(infinite_room.second.plane1_id);
    auto found_plane2 = plane_snapshot.find(infinite_room.second.plane2_id);

    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = dynamic_cast<g2o::VertexRoom*>(inf_room_map->second)
               ->estimate()
               .translation()(0);
    p1.y = dynamic_cast<g2o::VertexRoom*>(inf_room_map->second)
               ->estimate()
               .translation()(1);
    p1.z =
        dynamic_cast<g2o::VertexFloor*>(floor_map->second)->estimate().translation()(2);

    p2 = compute_plane_point(p1, (*found_plane1).second.cloud_seg_map);
    if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
    } else {
      infinite_room_line_marker.points.push_back(p1);
      infinite_room_line_marker.points.push_back(p2);
    }

    p3 = compute_plane_point(p1, (*found_plane2).second.cloud_seg_map);
    if (p3.x == 0 && p3.y == 0 && p3.z == 0) {
    } else {
      infinite_room_line_marker.points.push_back(p1);
      infinite_room_line_marker.points.push_back(p3);
    }
    markers.markers.push_back(infinite_room_line_marker);

    // y infinite_room cube
    visualization_msgs::msg::Marker infinite_room_pose_marker;
    infinite_room_pose_marker.pose.orientation.w = 1.0;
    infinite_room_pose_marker.scale.x = 0.5;
    infinite_room_pose_marker.scale.y = 0.5;
    infinite_room_pose_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    infinite_room_pose_marker.header.frame_id = rooms_layer_id;
    infinite_room_pose_marker.header.stamp = stamp;
    if (room_class == 0) infinite_room_pose_marker.ns = "x_infinite_room";
    if (room_class == 1) infinite_room_pose_marker.ns = "y_infinite_room";
    infinite_room_pose_marker.id = markers.markers.size();
    infinite_room_pose_marker.type = visualization_msgs::msg::Marker::CUBE;
    infinite_room_pose_marker.lifetime = marker_lifetime;
    infinite_room_pose_marker.color.r = 0.13;
    infinite_room_pose_marker.color.g = 0.54;
    infinite_room_pose_marker.color.b = 0.13;
    infinite_room_pose_marker.color.a = 1;
    infinite_room_pose_marker.pose.position.x = p1.x;
    infinite_room_pose_marker.pose.position.y = p1.y;
    infinite_room_pose_marker.pose.position.z = p1.z;

    Eigen::Quaterniond quat(
        dynamic_cast<g2o::VertexRoom*>(inf_room_map->second)->estimate().linear());
    infinite_room_pose_marker.pose.orientation.x = quat.x();
    infinite_room_pose_marker.pose.orientation.y = quat.y();
    infinite_room_pose_marker.pose.orientation.z = quat.z();
    infinite_room_pose_marker.pose.orientation.w = quat.w();
    markers.markers.push_back(infinite_room_pose_marker);
  }
}

}  // namespace s_graphs
