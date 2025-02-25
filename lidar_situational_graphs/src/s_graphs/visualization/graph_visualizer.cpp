#include <rclcpp/logger.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>

namespace s_graphs {

GraphVisualizer::GraphVisualizer(const rclcpp::Node::SharedPtr node,
                                 std::mutex& graph_mutex)
    : shared_graph_mutex(graph_mutex) {
  node_ptr_ = node.get();

  map_frame_id =
      node->get_parameter("map_frame_id").get_parameter_value().get<std::string>();

  line_color_r =
      node->get_parameter("line_color_r").get_parameter_value().get<double>();
  line_color_g =
      node->get_parameter("line_color_g").get_parameter_value().get<double>();
  line_color_b =
      node->get_parameter("line_color_b").get_parameter_value().get<double>();

  room_cube_color_r =
      node->get_parameter("room_cube_color_r").get_parameter_value().get<double>();
  room_cube_color_g =
      node->get_parameter("room_cube_color_g").get_parameter_value().get<double>();
  room_cube_color_b =
      node->get_parameter("room_cube_color_b").get_parameter_value().get<double>();

  floor_cube_color_r =
      node->get_parameter("floor_cube_color_r").get_parameter_value().get<double>();
  floor_cube_color_g =
      node->get_parameter("floor_cube_color_g").get_parameter_value().get<double>();
  floor_cube_color_b =
      node->get_parameter("floor_cube_color_b").get_parameter_value().get<double>();

  line_marker_size =
      node->get_parameter("line_marker_size").get_parameter_value().get<double>();
  kf_marker_size =
      node->get_parameter("kf_marker_size").get_parameter_value().get<double>();
  room_marker_size =
      node->get_parameter("room_marker_size").get_parameter_value().get<double>();
  floor_marker_size =
      node->get_parameter("floor_marker_size").get_parameter_value().get<double>();

  marker_duration_time =
      node->get_parameter("marker_duration").get_parameter_value().get<int>();

  std::string ns = node_ptr_->get_namespace();
  if (ns.length() > 1) {
    std::string ns_prefix = std::string(node_ptr_->get_namespace()).substr(1);
    map_frame_id = ns_prefix + "/" + map_frame_id;
  }

  keyframe_node_visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      "keyframes_layer", "/rviz_keyframe_node_visual_tools", node);
  keyframe_node_visual_tools->loadMarkerPub(false);
  keyframe_node_visual_tools->deleteAllMarkers();
  keyframe_node_visual_tools->enableBatchPublishing();
  keyframe_node_visual_tools->setAlpha(0.8);

  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

GraphVisualizer::~GraphVisualizer() {}

visualization_msgs::msg::MarkerArray
GraphVisualizer::visualize_floor_covisibility_graph(
    const rclcpp::Time& stamp,
    const g2o::SparseOptimizer* local_graph,
    std::vector<KeyFrame::Ptr> keyframes,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot,
    const std::unordered_map<int, InfiniteRooms> x_infinite_room_snapshot,
    const std::unordered_map<int, InfiniteRooms> y_infinite_room_snapshot,
    const std::unordered_map<int, Rooms> room_snapshot,
    const std::map<int, Floors> floors_vec) {
  visualization_msgs::msg::MarkerArray markers;
  rclcpp::Duration marker_duration =
      rclcpp::Duration::from_seconds(marker_duration_time);

  for (const auto& floor : floors_vec) {
    std::string keyframes_layer_id =
        "floor_" + std::to_string(floor.second.sequential_id) + "_keyframes_layer";

    std::string walls_layer_id =
        "floor_" + std::to_string(floor.second.sequential_id) + "_walls_layer";

    std::string rooms_layer_id =
        "floor_" + std::to_string(floor.second.sequential_id) + "_rooms_layer";

    std::string floors_layer_id =
        "floor_" + std::to_string(floor.second.sequential_id) + "_floors_layer";

    std::string ns = node_ptr_->get_namespace();
    if (ns.length() > 1) {
      std::string ns_prefix = std::string(node_ptr_->get_namespace()).substr(1);
      keyframes_layer_id = ns_prefix + "/" + keyframes_layer_id;
      walls_layer_id = ns_prefix + "/" + walls_layer_id;
      rooms_layer_id = ns_prefix + "/" + rooms_layer_id;
      floors_layer_id = ns_prefix + "/" + floors_layer_id;
    }

    std::vector<KeyFrame::Ptr> floor_keyframes;
    for (const auto& kf : keyframes) {
      if (kf->floor_level == floor.first) floor_keyframes.push_back(kf);
    }

    if (floor_keyframes.empty()) continue;
    int current_floor_level = floor.first;

    visualization_msgs::msg::Marker kf_marker =
        fill_kf_markers(local_graph, floor_keyframes, floors_vec);
    kf_marker.header.stamp = stamp;
    kf_marker.header.frame_id = keyframes_layer_id;
    kf_marker.ns = "floor_" + std::to_string(floor.second.sequential_id) + "_keyframes";
    kf_marker.id = markers.markers.size();
    kf_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_marker.pose.orientation.w = 1.0;
    kf_marker.scale.x = kf_marker.scale.y = kf_marker.scale.z = 0.3;
    kf_marker.lifetime = marker_duration;
    markers.markers.push_back(kf_marker);

    // keyframe edge markers
    visualization_msgs::msg::Marker kf_edge_marker =
        fill_kf_edge_markers(local_graph, floor_keyframes, floors_vec);
    kf_edge_marker.header.frame_id = keyframes_layer_id;
    kf_edge_marker.header.stamp = stamp;
    kf_edge_marker.ns =
        "floor_" + std::to_string(floor.second.sequential_id) + "_keyframe_edges";
    kf_edge_marker.id = markers.markers.size();
    kf_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    kf_edge_marker.pose.orientation.w = 1.0;
    kf_edge_marker.scale.x = line_marker_size;
    kf_edge_marker.lifetime = marker_duration;
    markers.markers.push_back(kf_edge_marker);

    // keyframe plane edge marker
    visualization_msgs::msg::Marker kf_plane_edge_marker =
        fill_kf_plane_markers(local_graph,
                              floor_keyframes,
                              x_plane_snapshot,
                              y_plane_snapshot,
                              hort_plane_snapshot,
                              keyframes_layer_id,
                              walls_layer_id);
    kf_plane_edge_marker.header.frame_id = keyframes_layer_id;
    kf_plane_edge_marker.header.stamp = stamp;
    kf_plane_edge_marker.ns =
        "floor_" + std::to_string(floor.second.sequential_id) + "_keyframe_plane_edges";
    kf_plane_edge_marker.id = markers.markers.size();
    kf_plane_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    kf_plane_edge_marker.pose.orientation.w = 1.0;
    kf_plane_edge_marker.scale.x = line_marker_size;
    markers.markers.push_back(kf_plane_edge_marker);

    // fil in the x_inf room marker
    this->fill_infinite_room(0,
                             stamp,
                             marker_duration,
                             local_graph,
                             x_plane_snapshot,
                             x_infinite_room_snapshot,
                             markers,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id,
                             current_floor_level);

    // fill in the y_inf room marker
    this->fill_infinite_room(1,
                             stamp,
                             marker_duration,
                             local_graph,
                             y_plane_snapshot,
                             y_infinite_room_snapshot,
                             markers,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id,
                             current_floor_level);

    this->fill_room(stamp,
                    marker_duration,
                    local_graph,
                    x_plane_snapshot,
                    y_plane_snapshot,
                    room_snapshot,
                    markers,
                    walls_layer_id,
                    rooms_layer_id,
                    floors_layer_id,
                    current_floor_level);

    this->fill_floor(stamp,
                     marker_duration,
                     local_graph,
                     x_infinite_room_snapshot,
                     y_infinite_room_snapshot,
                     room_snapshot,
                     floors_vec,
                     markers,
                     rooms_layer_id,
                     floors_layer_id,
                     current_floor_level);
  }

  return markers;
}

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
    const std::map<int, Floors> floors_vec) {
  visualization_msgs::msg::MarkerArray markers;

  // node markers
  double wall_vertex_h = 18;
  // lifetime
  rclcpp::Duration marker_lifetime = rclcpp::Duration::from_seconds(10);

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

  visualization_msgs::msg::Marker traj_marker =
      fill_kf_markers(local_graph, keyframes, floors_vec);
  traj_marker.header.frame_id = keyframes_layer_id;
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = markers.markers.size();
  traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.2;
  traj_marker.lifetime = marker_lifetime;
  markers.markers.push_back(traj_marker);

  // keyframe edge markers
  visualization_msgs::msg::Marker traj_edge_marker =
      fill_kf_edge_markers(local_graph, keyframes, floors_vec);
  traj_edge_marker.header.frame_id = keyframes_layer_id;
  traj_edge_marker.header.stamp = stamp;
  traj_edge_marker.ns = "keyframe_keyframe_edges";
  traj_edge_marker.id = markers.markers.size();
  traj_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  traj_edge_marker.lifetime = marker_lifetime;
  traj_edge_marker.pose.orientation.w = 1.0;
  traj_edge_marker.scale.x = 0.02;
  markers.markers.push_back(traj_edge_marker);

  // keyframe plane edge markers
  visualization_msgs::msg::Marker traj_plane_edge_marker =
      fill_kf_plane_markers(local_graph,
                            keyframes,
                            x_plane_snapshot,
                            y_plane_snapshot,
                            hort_plane_snapshot,
                            keyframes_layer_id,
                            walls_layer_id);
  traj_plane_edge_marker.header.frame_id = keyframes_layer_id;
  traj_plane_edge_marker.header.stamp = stamp;
  traj_plane_edge_marker.ns = "keyframe_plane_edges";
  traj_plane_edge_marker.id = markers.markers.size();
  traj_plane_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  traj_plane_edge_marker.lifetime = marker_lifetime;
  traj_plane_edge_marker.pose.orientation.w = 1.0;
  traj_plane_edge_marker.scale.x = 0.01;
  markers.markers.push_back(traj_plane_edge_marker);

  // Wall edge markers
  // TODO:HB include this in the previous for loop
  visualization_msgs::msg::Marker wall_center_marker;
  auto wall_edge_itr = local_graph->edges().begin();
  for (int i = 0; wall_edge_itr != local_graph->edges().end(); wall_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *wall_edge_itr;
    g2o::EdgeWall2Planes* edge_wall = dynamic_cast<g2o::EdgeWall2Planes*>(edge);
    if (edge_wall) {
      g2o::VertexWall* v1 = dynamic_cast<g2o::VertexWall*>(edge_wall->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[1]);
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[2]);
      Eigen::Vector3d wall_center = v1->estimate();

      wall_center_marker.ns = "wall_center_marker";
      wall_center_marker.header.frame_id = map_frame_id;
      wall_center_marker.header.stamp = stamp;
      wall_center_marker.id = markers.markers.size() + 1;
      wall_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      wall_center_marker.lifetime = marker_lifetime;
      wall_center_marker.color.r = line_color_r;
      wall_center_marker.color.g = line_color_r;
      wall_center_marker.color.b = line_color_r;
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
                           markers,
                           walls_layer_id,
                           rooms_layer_id,
                           floors_layer_id);

  // fill in the y_inf room marker
  this->fill_infinite_room(1,
                           stamp,
                           marker_lifetime,
                           local_graph,
                           y_plane_snapshot,
                           y_infinite_room_snapshot,
                           markers,
                           walls_layer_id,
                           rooms_layer_id,
                           floors_layer_id);

  this->fill_room(stamp,
                  marker_lifetime,
                  local_graph,
                  x_plane_snapshot,
                  y_plane_snapshot,
                  room_snapshot,
                  markers,
                  walls_layer_id,
                  rooms_layer_id,
                  floors_layer_id);

  this->fill_floor(stamp,
                   marker_lifetime,
                   local_graph,
                   x_infinite_room_snapshot,
                   y_infinite_room_snapshot,
                   room_snapshot,
                   floors_vec,
                   markers,
                   rooms_layer_id,
                   floors_layer_id);

  return markers;
}

std::vector<visualization_msgs::msg::MarkerArray> GraphVisualizer::visualize_a_graph(
    const rclcpp::Time& stamp,
    const g2o::SparseOptimizer* local_graph,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes_prior,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes_prior,
    std::unordered_map<int, Rooms> rooms_vec_prior,
    std::unordered_map<int, Rooms> rooms_vec,
    bool got_trans_prior2map_,
    const std::vector<DoorWays> doorways_vec_prior,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes) {
  std::vector<visualization_msgs::msg::MarkerArray> prior_markers_vec;
  visualization_msgs::msg::MarkerArray prior_x_plane_markers;
  visualization_msgs::msg::MarkerArray prior_y_plane_markers;
  visualization_msgs::msg::MarkerArray prior_wall_markers;
  visualization_msgs::msg::MarkerArray prior_wall_edge_markers;
  visualization_msgs::msg::MarkerArray prior_room_markers;
  visualization_msgs::msg::MarkerArray prior_room_edge_markers;
  visualization_msgs::msg::MarkerArray prior_doorway_markers;
  visualization_msgs::msg::MarkerArray prior_doorway_edge_markers;
  visualization_msgs::msg::MarkerArray wall_deviation_markers;
  visualization_msgs::msg::MarkerArray wall_center_markers;
  visualization_msgs::msg::MarkerArray wall_edge_deviation_markers;
  visualization_msgs::msg::MarkerArray room_deviation_markers;
  visualization_msgs::msg::MarkerArray room_edge_deviation_markers;
  visualization_msgs::msg::MarkerArray floor_markers;
  visualization_msgs::msg::MarkerArray floor_edge_markers;
  double plane_h = 15;
  double wall_vertex_h = 18;
  double prior_room_h = 22;
  double deviation_h = 16;

  int marker_id = prior_x_plane_markers.markers.size();

  for (const auto& [key, plane] : x_vert_planes_prior) {
    visualization_msgs::msg::Marker wall_visual_marker;
    if (!got_trans_prior2map_) {
      wall_visual_marker.header.frame_id = "prior_map";
    } else {
      wall_visual_marker.header.frame_id = map_frame_id;
    }
    wall_visual_marker.header.stamp = stamp;
    wall_visual_marker.ns = "prior_x_walls";
    wall_visual_marker.id = marker_id++;
    wall_visual_marker.type = visualization_msgs::msg::Marker::CUBE;

    wall_visual_marker.pose.position.x = plane.start_point.x();
    wall_visual_marker.pose.position.y = plane.start_point.y() + 0.5 * plane.length;
    wall_visual_marker.pose.position.z = plane_h;

    tf2::Quaternion rotation;
    rotation.setRPY(0, M_PI / 2, 0);  // 90 degrees in radians around the y-axis
    wall_visual_marker.pose.orientation.x = rotation.getX();
    wall_visual_marker.pose.orientation.y = rotation.getY();
    wall_visual_marker.pose.orientation.z = rotation.getZ();
    wall_visual_marker.pose.orientation.w = rotation.getW();

    wall_visual_marker.scale.x = 3.0;
    wall_visual_marker.scale.y = plane.length;
    wall_visual_marker.scale.z = 0.01;

    wall_visual_marker.color.r = plane.color[0] / 255;
    wall_visual_marker.color.g = plane.color[1] / 255;
    wall_visual_marker.color.b = plane.color[2] / 255;
    wall_visual_marker.color.a = 0.7;

    prior_x_plane_markers.markers.push_back(wall_visual_marker);
  }
  prior_markers_vec.push_back(prior_x_plane_markers);
  for (const auto& [key, plane] : y_vert_planes_prior) {
    visualization_msgs::msg::Marker wall_visual_marker;

    wall_visual_marker.header.frame_id =
        got_trans_prior2map_ ? map_frame_id : "prior_map";
    wall_visual_marker.header.stamp = stamp;
    wall_visual_marker.ns =
        "prior_y_vert_plane " + std::to_string(plane.plane_node->id());
    wall_visual_marker.id = marker_id++;
    wall_visual_marker.type = visualization_msgs::msg::Marker::CUBE;

    wall_visual_marker.pose.position.x = plane.start_point.x() + 0.5 * plane.length;
    wall_visual_marker.pose.position.y = plane.start_point.y();
    wall_visual_marker.pose.position.z = plane_h;

    tf2::Quaternion rotation;
    rotation.setRPY(0, M_PI / 2, 0);  // 90 degrees in radians around the y-axis
    wall_visual_marker.pose.orientation.x = rotation.getX();
    wall_visual_marker.pose.orientation.y = rotation.getY();
    wall_visual_marker.pose.orientation.z = rotation.getZ();
    wall_visual_marker.pose.orientation.w = rotation.getW();

    wall_visual_marker.scale.x = 3.0;
    wall_visual_marker.scale.y = 0.01;
    wall_visual_marker.scale.z = plane.length;

    wall_visual_marker.color.r = plane.color[0] / 255.0;
    wall_visual_marker.color.g = plane.color[1] / 255.0;
    wall_visual_marker.color.b = plane.color[2] / 255.0;
    wall_visual_marker.color.a = 1.0;

    prior_y_plane_markers.markers.push_back(wall_visual_marker);
  }
  prior_markers_vec.push_back(prior_y_plane_markers);

  if (!got_trans_prior2map_) {
    auto create_marker = [&](const std::string& ns,
                             int& id,
                             const Eigen::Matrix4d& pose,
                             double height,
                             const std::array<double, 4>& color) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "prior_map";
      marker.header.stamp = stamp;
      marker.ns = ns;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.pose.position.x = pose(0, 3);
      marker.pose.position.y = pose(1, 3);
      marker.pose.position.z = height;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = color[3];
      return marker;
    };

    auto create_edge_marker = [&](const std::string& ns,
                                  int& id,
                                  const geometry_msgs::msg::Point& start,
                                  const geometry_msgs::msg::Point& end,
                                  const std::array<double, 4>& color) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "prior_map";
      marker.header.stamp = stamp;
      marker.ns = ns;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = color[3];
      marker.points = {start, end};
      return marker;
    };

    for (const auto& [key, room] : rooms_vec_prior) {
      Eigen::Matrix4d room_pose = room.node->estimate().matrix();
      prior_room_markers.markers.push_back(create_marker(
          "prior_rooms", marker_id, room_pose, prior_room_h, {0.0, 0.7, 1.0, 1.0}));

      geometry_msgs::msg::Point room_point;
      room_point.x = room_pose(0, 3);
      room_point.y = room_pose(1, 3);
      room_point.z = prior_room_h;

      std::array<int, 4> plane_ids = {
          room.plane_x1_id, room.plane_x2_id, room.plane_y1_id, room.plane_y2_id};
      std::array<std::array<double, 4>, 4> edge_colors;
      edge_colors[0] = {1.0, 1.0, 1.0, 1.0};
      edge_colors[1] = {1.0, 1.0, 1.0, 1.0};
      edge_colors[2] = {1.0, 1.0, 1.0, 1.0};
      edge_colors[3] = {1.0, 1.0, 1.0, 1.0};

      for (int i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point plane_point;
        plane_point.z = 1.0 + plane_h;

        auto find_plane = [&](const auto& planes, int id) {
          auto it = std::find_if(planes.begin(), planes.end(), [id](const auto& p) {
            return p.second.id == id;
          });
          return (it != planes.end()) ? &(it->second) : nullptr;
        };

        if (auto x_plane = find_plane(x_vert_planes_prior, plane_ids[i])) {
          plane_point.x = x_plane->start_point.x();
          plane_point.y = room_pose(1, 3);
        } else if (auto y_plane = find_plane(y_vert_planes_prior, plane_ids[i])) {
          plane_point.x = room_pose(0, 3);
          plane_point.y = y_plane->start_point.y();
        } else {
          continue;  // Skip if plane not found
        }

        prior_room_edge_markers.markers.push_back(
            create_edge_marker("prior_room_to_plane_markers",
                               marker_id,
                               room_point,
                               plane_point,
                               edge_colors[i]));
      }
    }
    prior_markers_vec.push_back(prior_room_markers);
    prior_markers_vec.push_back(prior_room_edge_markers);
  }

  // Wall edge markers
  visualization_msgs::msg::Marker wall_center_marker;
  auto wall_edge_itr = local_graph->edges().begin();
  for (int i = 0; wall_edge_itr != local_graph->edges().end(); wall_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *wall_edge_itr;
    g2o::EdgeWall2Planes* edge_wall = dynamic_cast<g2o::EdgeWall2Planes*>(edge);
    if (edge_wall) {
      g2o::VertexWall* v1 = dynamic_cast<g2o::VertexWall*>(edge_wall->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[1]);
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[2]);
      Eigen::Vector3d wall_center = v1->estimate();

      wall_center_marker.ns = "wall_center_marker";
      wall_center_marker.header.frame_id = "prior_map";
      wall_center_marker.header.stamp = stamp;
      wall_center_marker.id = wall_center_markers.markers.size() + 1;
      wall_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
      wall_center_marker.color.r = 1.0;
      wall_center_marker.color.g = 1.0;
      wall_center_marker.color.b = 1.0;
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
      wall_center_markers.markers.push_back(wall_center_marker);
    }
  }
  prior_markers_vec.push_back(wall_center_markers);
  // for (int i = 0; i < doorways_vec_prior.size(); i++) {  // walls_x_coord.size()

  //   double r, g, b;
  //   visualization_msgs::msg::Marker prior_door_marker;
  //   prior_door_marker.header.frame_id = "prior_map";
  //   prior_door_marker.header.stamp = stamp;
  //   prior_door_marker.ns = "prior_doorways";
  //   prior_door_marker.id = marker_id++ + i;
  //   prior_door_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  //   prior_door_marker.mesh_resource = "package://is_graphs/meshes/door.dae";
  //   Eigen::Vector3d door_pose = doorways_vec_prior[i].door_pos_w;
  //   prior_door_marker.pose.position.x = door_pose.x();
  //   prior_door_marker.pose.position.y = door_pose.y();
  //   prior_door_marker.pose.position.z = prior_room_h;
  //   prior_door_marker.pose.orientation.x = 0;
  //   prior_door_marker.pose.orientation.y = 0;
  //   prior_door_marker.pose.orientation.z = 0.0;
  //   prior_door_marker.pose.orientation.w = 1;

  //   prior_door_marker.scale.y = 0.08;
  //   prior_door_marker.scale.x = 0.5;
  //   prior_door_marker.scale.z = 0.5;
  //   prior_door_marker.color.r = 193 / 255.0;
  //   prior_door_marker.color.g = 154 / 255.0;
  //   prior_door_marker.color.b = 107 / 255.0;
  //   prior_door_marker.color.a = 1.0;
  //   prior_markers.markers.push_back(prior_door_marker);
  //   if (!got_trans_prior2map_) {
  //     geometry_msgs::msg::Point point1, point2, point3;
  //     point1.x = door_pose.x();
  //     point1.y = door_pose.y();
  //     point1.z = prior_room_h;
  //     visualization_msgs::msg::Marker door_edge_plane_marker1;
  //     door_edge_plane_marker1.header.frame_id = "prior_map";
  //     door_edge_plane_marker1.header.stamp = stamp;
  //     door_edge_plane_marker1.ns = "prior_door_edges";
  //     door_edge_plane_marker1.id = marker_id++;
  //     door_edge_plane_marker1.type = visualization_msgs::msg::Marker::LINE_LIST;
  //     door_edge_plane_marker1.pose.orientation.w = 1.0;
  //     for (int j = 0; j < rooms_vec_prior.size(); j++) {
  //       if (rooms_vec_prior[j].prior_id == doorways_vec_prior[i].room1_id) {
  //         Eigen::Matrix4d room_pose =
  //         rooms_vec_prior[j].node->estimate().matrix(); point2.x = room_pose(0,
  //         3); point2.y = room_pose(1, 3); point2.z = prior_room_h;
  //       }
  //     }

  //     door_edge_plane_marker1.scale.x = 0.01;
  //     door_edge_plane_marker1.color.r = 0.0;
  //     door_edge_plane_marker1.color.g = 0.0;
  //     door_edge_plane_marker1.color.b = 0.0;
  //     door_edge_plane_marker1.color.a = 0.5;
  //     door_edge_plane_marker1.points.push_back(point1);
  //     door_edge_plane_marker1.points.push_back(point2);
  //     prior_markers.markers.push_back(door_edge_plane_marker1);

  //     visualization_msgs::msg::Marker door_edge_plane_marker2;
  //     door_edge_plane_marker2.header.frame_id = "prior_map";
  //     door_edge_plane_marker2.header.stamp = stamp;
  //     door_edge_plane_marker2.ns = "prior_door_edges";
  //     door_edge_plane_marker2.id = marker_id++;
  //     door_edge_plane_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
  //     door_edge_plane_marker2.pose.orientation.w = 1.0;
  //     for (int j = 0; j < rooms_vec_prior.size(); j++) {
  //       if (rooms_vec_prior[j].prior_id == doorways_vec_prior[i].room2_id) {
  //         Eigen::Matrix4d room_pose =
  //         rooms_vec_prior[j].node->estimate().matrix(); point3.x = room_pose(0,
  //         3); point3.y = room_pose(1, 3); point3.z = prior_room_h;
  //       }
  //     }
  //     door_edge_plane_marker2.scale.x = 0.01;
  //     door_edge_plane_marker2.color.r = 0.0;
  //     door_edge_plane_marker2.color.g = 0.0;
  //     door_edge_plane_marker2.color.b = 0.0;
  //     door_edge_plane_marker2.color.a = 0.5;
  //     door_edge_plane_marker2.points.push_back(point1);
  //     door_edge_plane_marker2.points.push_back(point3);
  //     prior_markers.markers.push_back(door_edge_plane_marker2);
  //   }

  //   if (got_trans_prior2map_) {
  //     geometry_msgs::msg::Point point1, point2, point3;
  //     point1.x = door_pose.x();
  //     point1.y = door_pose.y();
  //     point1.z = prior_room_h;
  //     visualization_msgs::msg::Marker door_edge_plane_marker1;
  //     door_edge_plane_marker1.header.frame_id = "prior_map";
  //     door_edge_plane_marker1.header.stamp = stamp;
  //     door_edge_plane_marker1.ns = "prior_door_edges";
  //     door_edge_plane_marker1.id = marker_id++;
  //     door_edge_plane_marker1.type = visualization_msgs::msg::Marker::LINE_LIST;
  //     door_edge_plane_marker1.pose.orientation.w = 1.0;
  //     for (int j = 0; j < rooms_vec.size(); j++) {
  //       if (rooms_vec[j].prior_id == doorways_vec_prior[i].room1_id) {
  //         Eigen::Matrix4d room_pose = rooms_vec[j].node->estimate().matrix();
  //         point2.x = room_pose(0, 3);
  //         point2.y = room_pose(1, 3);
  //         point2.z = prior_room_h;
  //       }
  //     }

  //     door_edge_plane_marker1.scale.x = 0.02;
  //     door_edge_plane_marker1.color.r = 0.0;
  //     door_edge_plane_marker1.color.g = 0.0;
  //     door_edge_plane_marker1.color.b = 0.0;
  //     door_edge_plane_marker1.color.a = 0.5;
  //     door_edge_plane_marker1.points.push_back(point1);
  //     door_edge_plane_marker1.points.push_back(point2);
  //     prior_markers.markers.push_back(door_edge_plane_marker1);

  //     visualization_msgs::msg::Marker door_edge_plane_marker2;
  //     door_edge_plane_marker2.header.frame_id = "prior_map";
  //     door_edge_plane_marker2.header.stamp = stamp;
  //     door_edge_plane_marker2.ns = "prior_door_edges";
  //     door_edge_plane_marker2.id = marker_id++;
  //     door_edge_plane_marker2.type = visualization_msgs::msg::Marker::LINE_LIST;
  //     door_edge_plane_marker2.pose.orientation.w = 1.0;
  //     for (int j = 0; j < rooms_vec.size(); j++) {
  //       if (rooms_vec[j].prior_id == doorways_vec_prior[i].room2_id) {
  //         Eigen::Matrix4d room_pose = rooms_vec[j].node->estimate().matrix();
  //         point3.x = room_pose(0, 3);
  //         point3.y = room_pose(1, 3);
  //         point3.z = prior_room_h;
  //       }
  //     }
  //     door_edge_plane_marker2.scale.x = 0.02;
  //     door_edge_plane_marker2.color.r = 0.0;
  //     door_edge_plane_marker2.color.g = 0.0;
  //     door_edge_plane_marker2.color.b = 0.0;
  //     door_edge_plane_marker2.color.a = 0.5;
  //     door_edge_plane_marker2.points.push_back(point1);
  //     door_edge_plane_marker2.points.push_back(point3);
  //     prior_markers.markers.push_back(door_edge_plane_marker2);
  //   }
  // }
  return prior_markers_vec;
}

void GraphVisualizer::visualize_compressed_graph(
    const rclcpp::Time& stamp,
    const int& current_floor_level,
    bool global_optimization,
    bool room_optimization,
    const g2o::SparseOptimizer* compressed_graph,
    const std::vector<KeyFrame::Ptr> keyframes,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot,
    const std::map<int, Floors>& floors_vec) {
  auto floor_itr = floors_vec.find(current_floor_level);
  Floors floor;
  if (floor_itr != floors_vec.end())
    floor = floors_vec.at(current_floor_level);
  else
    return;

  std::string keyframes_layer_id =
      "floor_" + std::to_string(floor.sequential_id) + "_keyframes_layer";

  keyframe_node_visual_tools->deleteAllMarkers();
  keyframe_node_visual_tools->setBaseFrame(keyframes_layer_id);

  rviz_visual_tools::Colors keyframe_edge_color, keyframe_plane_edge_color;
  rviz_visual_tools::Colors keyframe_node_color;

  if (global_optimization) {
    keyframe_node_color = rviz_visual_tools::ORANGE;
  } else if (room_optimization) {
    keyframe_node_color = rviz_visual_tools::RED;
  } else {
    keyframe_node_color = rviz_visual_tools::ORANGE;
  }

  std::vector<std::vector<KeyFrame::Ptr>> non_floor_kfs;
  non_floor_kfs.resize(floors_vec.size());
  for (const auto& kf : keyframes) {
    auto kf_map = compressed_graph->vertices().find(kf->id());
    if (kf_map == compressed_graph->vertices().end()) continue;

    if (kf->floor_level == current_floor_level) {
      const g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(kf_map->second);
      if (vertex_se3) {
        this->publish_cuboid(vertex_se3, keyframe_node_color);
      }
    } else {
      non_floor_kfs[floors_vec.at(kf->floor_level).sequential_id].push_back(kf);
    }
  }
  keyframe_node_visual_tools->trigger();

  int id = 0;
  for (int i = 0; i < non_floor_kfs.size(); ++i) {
    if (!non_floor_kfs[i].empty()) {
      keyframes_layer_id = "floor_" + std::to_string(i) + "_keyframes_layer";
      keyframe_node_visual_tools->setBaseFrame(keyframes_layer_id);

      for (const auto& kf : non_floor_kfs[i]) {
        auto kf_map = compressed_graph->vertices().find(kf->id());
        if (kf_map == compressed_graph->vertices().end()) continue;
        const g2o::VertexSE3* vertex_se3 =
            dynamic_cast<g2o::VertexSE3*>(kf_map->second);
        if (vertex_se3) {
          this->publish_cuboid(vertex_se3, keyframe_node_color);
        }
      }
      keyframe_node_visual_tools->trigger();
    }
  }

  return;
}

void GraphVisualizer::publish_cuboid(const g2o::VertexSE3* vertex_se3,
                                     rviz_visual_tools::Colors keyframe_node_color) {
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

Eigen::Isometry3d GraphVisualizer::compute_plane_pose(const VerticalPlanes& plane,
                                                      PointNormal& p_min,
                                                      PointNormal& p_max) {
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

visualization_msgs::msg::Marker GraphVisualizer::fill_kf_markers(
    const g2o::SparseOptimizer* local_graph,
    const std::vector<KeyFrame::Ptr> keyframes,
    const std::map<int, Floors> floors_vec) {
  visualization_msgs::msg::Marker kf_marker;
  kf_marker.points.resize(keyframes.size());
  kf_marker.colors.resize(keyframes.size());

  for (int i = 0; i < keyframes.size(); i++) {
    auto kf_map = local_graph->vertices().find(keyframes[i]->id());
    if (kf_map == local_graph->vertices().end()) continue;

    auto kf_node = dynamic_cast<g2o::VertexSE3*>(kf_map->second);
    Eigen::Vector3d pos = kf_node->estimate().translation();
    kf_marker.points[i].x = pos.x();
    kf_marker.points[i].y = pos.y();
    kf_marker.points[i].z = pos.z();

    auto current_key_data = dynamic_cast<OptimizationData*>(kf_node->userData());
    auto current_floor = floors_vec.find(keyframes[i]->floor_level);

    if (current_key_data) {
      bool marginalized = false;
      current_key_data->get_marginalized_info(marginalized);
      bool stair_keyframe = false;
      current_key_data->get_stair_node_info(stair_keyframe);
      if (marginalized) {
        kf_marker.colors[i].r = 0.0;
        kf_marker.colors[i].g = 0.0;
        kf_marker.colors[i].b = 0.0;
        kf_marker.colors[i].a = 1.0;
      } else if (stair_keyframe) {
        kf_marker.colors[i].r = 0.0;
        kf_marker.colors[i].g = 0.0;
        kf_marker.colors[i].b = 1.0;
        kf_marker.colors[i].a = 1.0;
      } else {
        kf_marker.colors[i].r = current_floor->second.color[0] / 255;
        kf_marker.colors[i].g = current_floor->second.color[1] / 255;
        kf_marker.colors[i].b = current_floor->second.color[2] / 255;
        kf_marker.colors[i].a = 1.0;
      }
    } else {
      kf_marker.colors[i].r = current_floor->second.color[0] / 255;
      kf_marker.colors[i].g = current_floor->second.color[1] / 255;
      kf_marker.colors[i].b = current_floor->second.color[2] / 255;
      kf_marker.colors[i].a = 1.0;
    }
  }
  return kf_marker;
}

visualization_msgs::msg::Marker GraphVisualizer::fill_kf_edge_markers(
    const g2o::SparseOptimizer* local_graph,
    const std::vector<KeyFrame::Ptr> keyframes,
    const std::map<int, Floors> floors_vec) {
  visualization_msgs::msg::Marker kf_edge_marker;

  for (int i = 0; i < keyframes.size(); i++) {
    auto kf_map = local_graph->vertices().find(keyframes[i]->id());
    if (kf_map == local_graph->vertices().end()) continue;
    auto kf_node = dynamic_cast<g2o::VertexSE3*>(kf_map->second);
    auto current_floor = floors_vec.find(keyframes[i]->floor_level);
    auto current_key_data = dynamic_cast<OptimizationData*>(kf_node->userData());
    bool stair_keyframe = false;

    if (current_key_data) {
      current_key_data->get_stair_node_info(stair_keyframe);
    }

    auto kf_edge_itr = kf_node->edges().begin();
    for (int i = 0; kf_edge_itr != kf_node->edges().end(); kf_edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *kf_edge_itr;
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
        kf_edge_marker.points.push_back(point1);
        kf_edge_marker.points.push_back(point2);

        double p1 = static_cast<double>(v1->id()) / local_graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / local_graph->vertices().size();

        if (!stair_keyframe) {
          std_msgs::msg::ColorRGBA color1, color2;
          color1.r = 0.0 * current_floor->second.color[0] / 255;
          color1.g = 0.0 * current_floor->second.color[1] / 255;
          color1.b = 0.0 * current_floor->second.color[2] / 255;
          color1.a = 1.0;

          color2.r = 0.0 * current_floor->second.color[0] / 255;
          color2.g = 0.0 * current_floor->second.color[1] / 255;
          color2.b = 0.0 * current_floor->second.color[2] / 255;
          color2.a = 1.0;
          kf_edge_marker.colors.push_back(color1);
          kf_edge_marker.colors.push_back(color2);
        } else {
          std_msgs::msg::ColorRGBA color1;
          color1.r = line_color_r;
          color1.g = line_color_g;
          color1.b = line_color_b;
          kf_edge_marker.colors.push_back(color1);
          kf_edge_marker.colors.push_back(color1);
        }
      }
    }
  }

  return kf_edge_marker;
}

visualization_msgs::msg::Marker GraphVisualizer::fill_kf_plane_markers(
    const g2o::SparseOptimizer* local_graph,
    const std::vector<KeyFrame::Ptr> keyframes,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, HorizontalPlanes>& hort_plane_snapshot,
    const std::string& keyframes_layer_id,
    const std::string& walls_layer_id) {
  visualization_msgs::msg::Marker kf_plane_edge_marker;

  for (int i = 0; i < keyframes.size(); i++) {
    auto kf_map = local_graph->vertices().find(keyframes[i]->id());
    if (kf_map == local_graph->vertices().end()) continue;
    auto kf_node = dynamic_cast<g2o::VertexSE3*>(kf_map->second);

    auto kf_edge_itr = kf_node->edges().begin();
    for (int i = 0; kf_edge_itr != kf_node->edges().end(); kf_edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *kf_edge_itr;
      g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if (edge_se3_plane) {
        g2o::VertexSE3* v1 =
            dynamic_cast<g2o::VertexSE3*>(edge_se3_plane->vertices()[0]);
        g2o::VertexPlane* v2 =
            dynamic_cast<g2o::VertexPlane*>(edge_se3_plane->vertices()[1]);

        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2;

        float r = 0, g = 0, b = 0.0;
        pcl::CentroidPoint<PointNormal> centroid;
        if (fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(1)) &&
            fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(2))) {
          pt2 = compute_plane_centroid<VerticalPlanes>(v2->id(), x_plane_snapshot);
          r = 0.0;
        } else if (fabs(v2->estimate().normal()(1)) >
                       fabs(v2->estimate().normal()(0)) &&
                   fabs(v2->estimate().normal()(1)) >
                       fabs(v2->estimate().normal()(2))) {
          pt2 = compute_plane_centroid<VerticalPlanes>(v2->id(), y_plane_snapshot);
          b = 0.0;
        } else if (fabs(v2->estimate().normal()(2)) >
                       fabs(v2->estimate().normal()(0)) &&
                   fabs(v2->estimate().normal()(2)) >
                       fabs(v2->estimate().normal()(1))) {
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
        if (tf_buffer->canTransform(keyframes_layer_id,
                                    point2_stamped.header.frame_id,
                                    tf2::TimePointZero)) {
          tf_buffer->transform(point2_stamped,
                               point2_stamped_transformed,
                               keyframes_layer_id,
                               tf2::TimePointZero,
                               walls_layer_id);
        } else
          continue;

        point2 = point2_stamped_transformed.point;
        kf_plane_edge_marker.points.push_back(point1);
        kf_plane_edge_marker.points.push_back(point2);

        std_msgs::msg::ColorRGBA color1, color2;
        color1.r = line_color_r;
        color1.g = line_color_g;
        color1.b = line_color_b;
        color1.a = 0.1;

        color2.r = line_color_r;
        color2.g = line_color_b;
        color2.b = line_color_g;
        color2.a = 0.1;
        kf_plane_edge_marker.colors.push_back(color1);
        kf_plane_edge_marker.colors.push_back(color2);
      }
    }
  }

  return kf_plane_edge_marker;
}

visualization_msgs::msg::Marker GraphVisualizer::fill_cloud_makers(
    const std::vector<KeyFrame::Ptr> keyframes) {
  visualization_msgs::msg::Marker cloud_marker;
  cloud_marker.pose.orientation.w = 1.0;
  cloud_marker.scale.x = 0.05;
  cloud_marker.scale.y = 0.05;
  cloud_marker.scale.z = 0.05;
  cloud_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  std_msgs::msg::ColorRGBA color;
  color.r = 211.0 / 255;
  color.g = 215.0 / 255;
  color.b = 207.0 / 255;
  color.a = 0.1;

  for (const auto& kf : keyframes) {
    Eigen::Matrix4f pose = kf->estimate().matrix().cast<float>();
    for (const auto& src_pt : kf->cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      geometry_msgs::msg::Point point;
      point.x = dst_pt.x;
      point.y = dst_pt.y;
      point.z = dst_pt.z;
      cloud_marker.points.push_back(point);
      cloud_marker.colors.push_back(color);
    }
  }
  return cloud_marker;
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
    color.a = 0.1;

    std::lock_guard<std::mutex> lock(shared_graph_mutex);
    if (plane.second.cloud_seg_map == nullptr) continue;

    for (int j = 0; j < plane.second.cloud_seg_map->points.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = plane.second.cloud_seg_map->points[j].x;
      point.y = plane.second.cloud_seg_map->points[j].y;
      point.z = plane.second.cloud_seg_map->points[j].z;
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
    std::lock_guard<std::mutex> lock(shared_graph_mutex);
    if (!plane->second.cloud_seg_map || plane->second.cloud_seg_map->points.empty()) {
      return pt;
    }

    double x = 0, y = 0, z = 0;
    for (const auto& point : plane->second.cloud_seg_map->points) {
      x += point.x;
      y += point.y;
      z += point.z;
    }
    x = x / plane->second.cloud_seg_map->points.size();
    y = y / plane->second.cloud_seg_map->points.size();
    z = z / plane->second.cloud_seg_map->points.size();
    pt = Eigen::Vector3d(x, y, z);
  }

  return pt;
}

geometry_msgs::msg::Point GraphVisualizer::compute_plane_point(
    geometry_msgs::msg::Point room_p1,
    const pcl::PointCloud<PointNormal>::Ptr cloud_seg_map,
    const std::string& walls_layer_id,
    const std::string& rooms_layer_id,
    const std::string& floors_layer_id) {
  float min_dist_plane1 = 100;
  geometry_msgs::msg::Point plane_p2;
  plane_p2.x = plane_p2.y = plane_p2.z = 0;

  std::lock_guard<std::mutex> lock(shared_graph_mutex);

  if (cloud_seg_map->points.empty()) {
    return plane_p2;
  }

  for (const auto& point : cloud_seg_map->points) {
    geometry_msgs::msg::Point p_tmp;
    p_tmp.x = point.x;
    p_tmp.y = point.y;
    p_tmp.z = point.z;

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
  if (tf_buffer->canTransform(
          rooms_layer_id, point2_stamped.header.frame_id, tf2::TimePointZero)) {
    tf_buffer->transform(point2_stamped,
                         point2_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);
  }

  return point2_stamped_transformed.point;
}

geometry_msgs::msg::Point GraphVisualizer::compute_room_point(
    geometry_msgs::msg::Point room_p1,
    const std::string& rooms_layer_id,
    const std::string& floors_layer_id) {
  geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
  point2_stamped.header.frame_id = rooms_layer_id;
  point2_stamped.point.x = room_p1.x;
  point2_stamped.point.y = room_p1.y;
  point2_stamped.point.z = room_p1.z;

  // convert point p2 to rooms_layer_id currently it is map_frame_id
  if (tf_buffer->canTransform(
          floors_layer_id, point2_stamped.header.frame_id, tf2::TimePointZero)) {
    tf_buffer->transform(point2_stamped,
                         point2_stamped_transformed,
                         floors_layer_id,
                         tf2::TimePointZero,
                         rooms_layer_id);
  }
  room_p1 = point2_stamped_transformed.point;

  return room_p1;
}

void GraphVisualizer::fill_infinite_room(
    const int& room_class,
    const rclcpp::Time& stamp,
    rclcpp::Duration marker_lifetime,
    const g2o::SparseOptimizer* local_graph,
    const std::unordered_map<int, VerticalPlanes>& plane_snapshot,
    const std::unordered_map<int, InfiniteRooms>& infinite_room_snapshot,
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& walls_layer_id,
    const std::string& rooms_layer_id,
    const std::string& floors_layer_id,
    const int& current_floor_level) {
  // fill in the line marker
  for (const auto& infinite_room : infinite_room_snapshot) {
    if (infinite_room.second.floor_level != current_floor_level) continue;

    auto inf_room_map = local_graph->vertices().find(infinite_room.first);
    if (inf_room_map == local_graph->vertices().end()) continue;

    auto floor_map = local_graph->vertices().find(infinite_room.second.floor_level);
    if (floor_map == local_graph->vertices().end()) {
      std::cout << "floor node not found for inf room" << std::endl;
      continue;
    }

    visualization_msgs::msg::Marker infinite_room_line_marker;
    infinite_room_line_marker.scale.x = line_marker_size;
    infinite_room_line_marker.pose.orientation.w = 1.0;
    if (room_class == 0) infinite_room_line_marker.ns = "infinite_room_x_lines";
    if (room_class == 1) infinite_room_line_marker.ns = "infinite_room_y_lines";
    infinite_room_line_marker.header.frame_id = rooms_layer_id;
    infinite_room_line_marker.header.stamp = stamp;
    infinite_room_line_marker.id = markers.markers.size() + 1;
    infinite_room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    infinite_room_line_marker.lifetime = marker_lifetime;
    infinite_room_line_marker.color.r = line_color_r;
    infinite_room_line_marker.color.g = line_color_g;
    infinite_room_line_marker.color.b = line_color_b;
    infinite_room_line_marker.color.a = 0.5;

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

    p2 = compute_plane_point(p1,
                             (*found_plane1).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
    } else {
      infinite_room_line_marker.points.push_back(p1);
      infinite_room_line_marker.points.push_back(p2);
    }

    p3 = compute_plane_point(p1,
                             (*found_plane2).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p3.x == 0 && p3.y == 0 && p3.z == 0) {
    } else {
      infinite_room_line_marker.points.push_back(p1);
      infinite_room_line_marker.points.push_back(p3);
    }
    markers.markers.push_back(infinite_room_line_marker);

    // y infinite_room cube
    visualization_msgs::msg::Marker infinite_room_pose_marker;
    infinite_room_pose_marker.pose.orientation.w = 1.0;
    infinite_room_pose_marker.scale.x = room_marker_size;
    infinite_room_pose_marker.scale.y = room_marker_size;
    infinite_room_pose_marker.scale.z = room_marker_size;
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

void GraphVisualizer::fill_room(
    const rclcpp::Time& stamp,
    rclcpp::Duration marker_lifetime,
    const g2o::SparseOptimizer* local_graph,
    const std::unordered_map<int, VerticalPlanes>& x_plane_snapshot,
    const std::unordered_map<int, VerticalPlanes>& y_plane_snapshot,
    const std::unordered_map<int, Rooms>& rooms_snapshot,
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& walls_layer_id,
    const std::string& rooms_layer_id,
    const std::string& floors_layer_id,
    const int& current_floor_level) {
  // room markers
  for (const auto& room : rooms_snapshot) {
    if (room.second.floor_level != current_floor_level) continue;

    auto room_map = local_graph->vertices().find(room.first);
    if (room_map == local_graph->vertices().end()) continue;

    auto floor_map = local_graph->vertices().find(room.second.floor_level);
    if (floor_map == local_graph->vertices().end()) {
      std::cout << "floor node not found for room" << std::endl;
      continue;
    }

    // fill in the line marker
    visualization_msgs::msg::Marker room_line_marker;
    room_line_marker.scale.x = line_marker_size;
    room_line_marker.pose.orientation.w = 1.0;
    room_line_marker.ns = "rooms_line";
    room_line_marker.header.frame_id = rooms_layer_id;
    room_line_marker.header.stamp = stamp;
    room_line_marker.id = markers.markers.size() + 1;
    room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    room_line_marker.lifetime = marker_lifetime;
    room_line_marker.color.r = line_color_r;
    room_line_marker.color.g = line_color_g;
    room_line_marker.color.b = line_color_b;
    room_line_marker.color.a = 0.5;
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

    p2 = compute_plane_point(p1,
                             (*found_planex1).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p2);
    }

    p3 = compute_plane_point(p1,
                             (*found_planex2).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p3.x == 0 && p3.y == 0 && p3.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p3);
    }

    p4 = compute_plane_point(p1,
                             (*found_planey1).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p4.x == 0 && p4.y == 0 && p4.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p4);
    }

    p5 = compute_plane_point(p1,
                             (*found_planey2).second.cloud_seg_map,
                             walls_layer_id,
                             rooms_layer_id,
                             floors_layer_id);
    if (p5.x == 0 && p5.y == 0 && p5.z == 0) {
    } else {
      room_line_marker.points.push_back(p1);
      room_line_marker.points.push_back(p5);
    }
    markers.markers.push_back(room_line_marker);

    // fill the pose marker
    visualization_msgs::msg::Marker room_marker;
    room_marker.scale.x = room_marker_size;
    room_marker.scale.y = room_marker_size;
    room_marker.scale.z = room_marker_size;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = rooms_layer_id;
    room_marker.header.stamp = stamp;
    room_marker.ns = "rooms";
    room_marker.id = markers.markers.size();
    room_marker.type = visualization_msgs::msg::Marker::CUBE;
    room_marker.color.r = room_cube_color_r;
    room_marker.color.g = room_cube_color_g;
    room_marker.color.b = room_cube_color_b;
    room_marker.color.a = 1;
    room_marker.lifetime = marker_lifetime;
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
}

void GraphVisualizer::fill_floor(
    const rclcpp::Time& stamp,
    rclcpp::Duration marker_lifetime,
    const g2o::SparseOptimizer* local_graph,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_inf_rooms_snapshot,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_inf_rooms_snapshot,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_snapshot,
    const std::map<int, s_graphs::Floors>& floors_snapshot,
    visualization_msgs::msg::MarkerArray& markers,
    const std::string& rooms_layer_id,
    const std::string& floors_layer_id,
    const int& current_floor_level) {
  for (const auto& floor : floors_snapshot) {
    if (floor.first != current_floor_level) continue;

    if (floor.first != -1) {
      auto floor_map = local_graph->vertices().find(floor.first);
      if (floor_map == local_graph->vertices().end()) {
        continue;
      }

      visualization_msgs::msg::Marker floor_marker;
      floor_marker.pose.orientation.w = 1.0;
      floor_marker.scale.x = floor_marker_size;
      floor_marker.scale.y = floor_marker_size;
      floor_marker.scale.z = floor_marker_size;
      // plane_marker.points.resize(vert_planes.size());
      floor_marker.header.frame_id = floors_layer_id;
      floor_marker.header.stamp = stamp;
      floor_marker.ns = "floors";
      floor_marker.id = markers.markers.size();
      floor_marker.type = visualization_msgs::msg::Marker::CUBE;
      floor_marker.lifetime = marker_lifetime;
      floor_marker.color.r = floor_cube_color_r;
      floor_marker.color.g = floor_cube_color_g;
      floor_marker.color.b = floor_cube_color_b;
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
      floor_line_marker.scale.x = line_marker_size;
      floor_line_marker.pose.orientation.w = 1.0;
      floor_line_marker.ns = "floor_lines";
      floor_line_marker.header.frame_id = floors_layer_id;
      floor_line_marker.header.stamp = stamp;
      floor_line_marker.id = markers.markers.size() + 1;
      floor_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      floor_line_marker.lifetime = marker_lifetime;
      floor_line_marker.color.r = line_color_r;
      floor_line_marker.color.g = line_color_r;
      floor_line_marker.color.b = line_color_r;
      floor_line_marker.color.a = 0.5;

      for (const auto& room : rooms_snapshot) {
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
        p2 = compute_room_point(p2, rooms_layer_id, floors_layer_id);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& x_infinite_room : x_inf_rooms_snapshot) {
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
        p2 = compute_room_point(p2, rooms_layer_id, floors_layer_id);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& y_infinite_room : y_inf_rooms_snapshot) {
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
        p2 = compute_room_point(p2, rooms_layer_id, floors_layer_id);

        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      markers.markers.push_back(floor_marker);
      markers.markers.push_back(floor_line_marker);
    }
  }
}

}  // namespace s_graphs
