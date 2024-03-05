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

// SPDX-License-Identifier: BSD-2-Clause

#include <rclcpp/logger.hpp>
#include <s_graphs/visualization/graph_visualizer.hpp>

namespace s_graphs {

GraphVisualizer::GraphVisualizer(const rclcpp::Node::SharedPtr node) {
  node_ptr_ = node.get();

  map_frame_id =
      node->get_parameter("map_frame_id").get_parameter_value().get<std::string>();
  color_r = node->get_parameter("color_r").get_parameter_value().get<double>();
  color_g = node->get_parameter("color_g").get_parameter_value().get<double>();
  color_b = node->get_parameter("color_b").get_parameter_value().get<double>();

  std::string ns = node_ptr_->get_namespace();
  if (ns.length() > 1) {
    std::string ns_prefix = std::string(node_ptr_->get_namespace()).substr(1);
    map_frame_id = ns_prefix + "/" + map_frame_id;
  }

  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

GraphVisualizer::~GraphVisualizer() {}

/**
 * @brief create visualization marker
 * @param stamp
 * @return
 */
std::vector<visualization_msgs::msg::MarkerArray> GraphVisualizer::create_marker_array(
    const rclcpp::Time& stamp,
    const g2o::SparseOptimizer* local_graph,
    const std::vector<VerticalPlanes>& x_plane_snapshot,
    const std::vector<VerticalPlanes>& y_plane_snapshot,
    const std::vector<HorizontalPlanes>& hort_plane_snapshot,
    std::vector<InfiniteRooms> x_infinite_room_snapshot,
    std::vector<InfiniteRooms> y_infinite_room_snapshot,
    std::vector<Rooms> room_snapshot,
    double loop_detector_radius,
    std::vector<KeyFrame::Ptr> keyframes,
    std::vector<Floors> floors_vec) {
  std::vector<visualization_msgs::msg::MarkerArray> markers_vec;
  visualization_msgs::msg::MarkerArray keyframe_markers;
  visualization_msgs::msg::MarkerArray trj_edge_markers;
  visualization_msgs::msg::MarkerArray x_plane_markers;
  visualization_msgs::msg::MarkerArray y_plane_markers;
  visualization_msgs::msg::MarkerArray traj_plane_edge_markers;
  visualization_msgs::msg::MarkerArray hort_plane_markers;
  visualization_msgs::msg::MarkerArray x_infinite_room_markers;
  visualization_msgs::msg::MarkerArray y_infinite_room_markers;
  visualization_msgs::msg::MarkerArray x_infinite_room_edge_markers;
  visualization_msgs::msg::MarkerArray y_infinite_room_edge_markers;
  visualization_msgs::msg::MarkerArray room_markers;
  visualization_msgs::msg::MarkerArray room_edge_markers;
  visualization_msgs::msg::MarkerArray loop_close_markers;
  visualization_msgs::msg::MarkerArray floor_markers;
  visualization_msgs::msg::MarkerArray floor_edge_markers;

  // markers.markers.resize(11);

  // node markers
  double wall_vertex_h = 18;
  rclcpp::Duration duration_planes = rclcpp::Duration::from_seconds(5);

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

  for (int i = 0; i < keyframes.size(); i++) {
    visualization_msgs::msg::Marker traj_marker;
    traj_marker.header.frame_id = keyframes_layer_id;
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "kf " + std::to_string(keyframes[i]->node->id());
    traj_marker.id = keyframe_markers.markers.size();
    traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
    geometry_msgs::msg::Point point1;
    point1.x = pos.x();
    point1.y = pos.y();
    point1.z = pos.z();
    traj_marker.points.push_back(point1);

    double p = static_cast<double>(i) / keyframes.size();
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0 - p;
    color.g = p;
    color.b = 0.0;
    color.a = 1.0;
    traj_marker.colors.push_back(color);
    keyframe_markers.markers.push_back(traj_marker);
  }
  markers_vec.push_back(keyframe_markers);

  auto traj_edge_itr = local_graph->edges().begin();
  for (int i = 0; traj_edge_itr != local_graph->edges().end(); traj_edge_itr++, i++) {
    // keyframe edge markers
    visualization_msgs::msg::Marker traj_edge_marker;
    traj_edge_marker.header.frame_id = keyframes_layer_id;
    traj_edge_marker.header.stamp = stamp;
    traj_edge_marker.ns = "keyframe_keyframe_edge " + std::to_string(i);
    traj_edge_marker.id = trj_edge_markers.markers.size();
    traj_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    traj_edge_marker.pose.orientation.w = 1.0;
    traj_edge_marker.scale.x = 0.05;

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
      color1.r = 1.0 - p1;
      color1.g = p1;
      color1.a = 1.0;

      color2.r = 1.0 - p2;
      color2.g = p2;
      color2.a = 1.0;
      traj_edge_marker.colors.push_back(color1);
      traj_edge_marker.colors.push_back(color2);

      trj_edge_markers.markers.push_back(traj_edge_marker);
    }
  }
  markers_vec.push_back(trj_edge_markers);

  auto traj_plane_edge_itr = local_graph->edges().begin();
  for (int i = 0; traj_plane_edge_itr != local_graph->edges().end();
       traj_plane_edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *traj_plane_edge_itr;
    g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);

    // keyframe plane edge markers
    visualization_msgs::msg::Marker traj_plane_edge_marker;
    traj_plane_edge_marker.header.frame_id = keyframes_layer_id;
    traj_plane_edge_marker.header.stamp = stamp;

    traj_plane_edge_marker.id = traj_plane_edge_markers.markers.size();
    traj_plane_edge_marker.lifetime = duration_planes;
    traj_plane_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    traj_plane_edge_marker.pose.orientation.w = 1.0;
    traj_plane_edge_marker.scale.x = 0.005;

    if (edge_plane) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(edge_plane->vertices()[1]);
      traj_plane_edge_marker.ns = "keyframe_plane_edge " + std::to_string(v1->id());
      if (!v1 || !v2) continue;

      Eigen::Vector3d pt1 = v1->estimate().translation();
      Eigen::Vector3d pt2;

      float r = 0, g = 0, b = 0.0;
      pcl::CentroidPoint<PointNormal> centroid;
      if (fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(1)) &&
          fabs(v2->estimate().normal()(0)) > fabs(v2->estimate().normal()(2))) {
        for (auto x_plane : x_plane_snapshot) {
          if (x_plane.id == v2->id()) {
            double x = 0, y = 0, z = 0;
            for (int p = 0; p < x_plane.cloud_seg_map->points.size(); ++p) {
              x += x_plane.cloud_seg_map->points[p].x;
              y += x_plane.cloud_seg_map->points[p].y;
              z += x_plane.cloud_seg_map->points[p].z;
            }
            x = x / x_plane.cloud_seg_map->points.size();
            y = y / x_plane.cloud_seg_map->points.size();
            z = z / x_plane.cloud_seg_map->points.size();
            pt2 = Eigen::Vector3d(x, y, z);
          }
        }
        r = 0.0;
      } else if (fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(0)) &&
                 fabs(v2->estimate().normal()(1)) > fabs(v2->estimate().normal()(2))) {
        for (auto y_plane : y_plane_snapshot) {
          if (y_plane.id == v2->id()) {
            double x = 0, y = 0, z = 0;
            for (int p = 0; p < y_plane.cloud_seg_map->points.size(); ++p) {
              x += y_plane.cloud_seg_map->points[p].x;
              y += y_plane.cloud_seg_map->points[p].y;
              z += y_plane.cloud_seg_map->points[p].z;
            }
            x = x / y_plane.cloud_seg_map->points.size();
            y = y / y_plane.cloud_seg_map->points.size();
            z = z / y_plane.cloud_seg_map->points.size();
            pt2 = Eigen::Vector3d(x, y, z);
          }
        }
        b = 0.0;
      } else if (fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(0)) &&
                 fabs(v2->estimate().normal()(2)) > fabs(v2->estimate().normal()(1))) {
        for (auto h_plane : hort_plane_snapshot) {
          if (h_plane.id == v2->id()) {
            double x = 0, y = 0, z = 0;
            for (int p = 0; p < h_plane.cloud_seg_map->points.size(); ++p) {
              x += h_plane.cloud_seg_map->points[p].x;
              y += h_plane.cloud_seg_map->points[p].y;
              z += h_plane.cloud_seg_map->points[p].z;
            }
            x = x / h_plane.cloud_seg_map->points.size();
            y = y / h_plane.cloud_seg_map->points.size();
            z = z / h_plane.cloud_seg_map->points.size();
            pt2 = Eigen::Vector3d(x, y, z);
          }
        }
        r = 0;
        g = 0.0;
      } else
        continue;

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

      traj_plane_edge_markers.markers.push_back(traj_plane_edge_marker);
    }
  }
  markers_vec.push_back(traj_plane_edge_markers);

  // Wall edge markers
  // visualization_msgs::msg::Marker wall_center_marker;
  // auto wall_edge_itr = local_graph->edges().begin();
  // for (int i = 0; wall_edge_itr != local_graph->edges().end(); wall_edge_itr++, i++)
  // {
  //   g2o::HyperGraph::Edge* edge = *wall_edge_itr;
  //   g2o::EdgeWall2Planes* edge_wall = dynamic_cast<g2o::EdgeWall2Planes*>(edge);
  //   if (edge_wall) {
  //     g2o::VertexWallXYZ* v1 =
  //         dynamic_cast<g2o::VertexWallXYZ*>(edge_wall->vertices()[0]);
  //     g2o::VertexPlane* v2 =
  //     dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[1]); g2o::VertexPlane* v3
  //     = dynamic_cast<g2o::VertexPlane*>(edge_wall->vertices()[2]); Eigen::Vector3d
  //     wall_center = v1->estimate();

  //     wall_center_marker.ns = "wall_center_marker";
  //     wall_center_marker.header.frame_id = map_frame_id;
  //     wall_center_marker.header.stamp = stamp;
  //     wall_center_marker.id = markers.markers.size() + 1;
  //     wall_center_marker.type = visualization_msgs::msg::Marker::SPHERE;
  //     wall_center_marker.color.r = color_r;
  //     wall_center_marker.color.g = color_g;
  //     wall_center_marker.color.b = color_b;
  //     wall_center_marker.color.a = 1.0;
  //     wall_center_marker.scale.x = 0.3;
  //     wall_center_marker.scale.y = 0.3;
  //     wall_center_marker.scale.z = 0.3;
  //     wall_center_marker.pose.position.x = wall_center.x();
  //     wall_center_marker.pose.position.y = wall_center.y();
  //     wall_center_marker.pose.position.z = wall_vertex_h;
  //     wall_center_marker.pose.orientation.x = 0.0;
  //     wall_center_marker.pose.orientation.y = 0.0;
  //     wall_center_marker.pose.orientation.z = 0.0;
  //     wall_center_marker.pose.orientation.w = 1.0;
  //     markers.markers.push_back(wall_center_marker);
  //   }
  // }

  // sphere
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = keyframes_layer_id;
  sphere_marker.header.stamp = stamp;
  sphere_marker.ns = "loop_close_radius";
  sphere_marker.id = loop_close_markers.markers.size();
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;

  if (!keyframes.empty()) {
    Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
    sphere_marker.pose.position.x = pos.x();
    sphere_marker.pose.position.y = pos.y();
    sphere_marker.pose.position.z = pos.z();
  }
  sphere_marker.pose.orientation.w = 1.0;
  sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z =
      loop_detector_radius;

  sphere_marker.color.r = 1.0;
  sphere_marker.color.a = 0.3;
  loop_close_markers.markers.push_back(sphere_marker);

  // x vertical plane markers
  // make a separate marker for each plane to have different colors and push all of them
  // in markers_vec

  for (int k = 0; k < x_plane_snapshot.size(); k++) {
    visualization_msgs::msg::Marker x_vert_plane_marker;
    x_vert_plane_marker.pose.orientation.w = 1.0;
    x_vert_plane_marker.scale.x = 0.05;
    x_vert_plane_marker.scale.y = 0.05;
    x_vert_plane_marker.scale.z = 0.05;
    // plane_marker.points.resize(vert_planes.size());
    x_vert_plane_marker.header.frame_id = walls_layer_id;
    x_vert_plane_marker.header.stamp = stamp;
    x_vert_plane_marker.ns =
        "x_vert_plane " + std::to_string(x_plane_snapshot[k].plane_node->id());
    x_vert_plane_marker.id = x_plane_markers.markers.size();
    x_vert_plane_marker.lifetime = duration_planes;
    x_vert_plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

    std_msgs::msg::ColorRGBA color;
    color.r = x_plane_snapshot[k].color[0] / 255;
    color.g = x_plane_snapshot[k].color[1] / 255;
    color.b = x_plane_snapshot[k].color[2] / 255;
    color.a = 0.5;
    for (size_t j = 0; j < x_plane_snapshot[k].cloud_seg_map->size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = x_plane_snapshot[k].cloud_seg_map->points[j].x;
      point.y = x_plane_snapshot[k].cloud_seg_map->points[j].y;
      point.z = x_plane_snapshot[k].cloud_seg_map->points[j].z;
      x_vert_plane_marker.points.push_back(point);
      x_vert_plane_marker.colors.push_back(color);
    }

    x_plane_markers.markers.push_back(x_vert_plane_marker);
  }
  markers_vec.push_back(x_plane_markers);
  // y vertical plane markers
  for (int k = 0; k < y_plane_snapshot.size(); k++) {
    visualization_msgs::msg::Marker y_vert_plane_marker;
    y_vert_plane_marker.pose.orientation.w = 1.0;
    y_vert_plane_marker.scale.x = 0.05;
    y_vert_plane_marker.scale.y = 0.05;
    y_vert_plane_marker.scale.z = 0.05;
    // plane_marker.points.resize(vert_planes.size());
    y_vert_plane_marker.header.frame_id = walls_layer_id;
    y_vert_plane_marker.header.stamp = stamp;
    y_vert_plane_marker.ns =
        "y_vert_plane " + std::to_string(y_plane_snapshot[k].plane_node->id());
    y_vert_plane_marker.id = y_plane_markers.markers.size();
    y_vert_plane_marker.lifetime = duration_planes;
    y_vert_plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

    std_msgs::msg::ColorRGBA color;
    color.r = y_plane_snapshot[k].color[0] / 255;
    color.g = y_plane_snapshot[k].color[1] / 255;
    color.b = y_plane_snapshot[k].color[2] / 255;
    color.a = 0.5;
    for (size_t j = 0; j < y_plane_snapshot[k].cloud_seg_map->size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = y_plane_snapshot[k].cloud_seg_map->points[j].x;
      point.y = y_plane_snapshot[k].cloud_seg_map->points[j].y;
      point.z = y_plane_snapshot[k].cloud_seg_map->points[j].z;
      y_vert_plane_marker.points.push_back(point);
      y_vert_plane_marker.colors.push_back(color);
    }

    y_plane_markers.markers.push_back(y_vert_plane_marker);
  }
  markers_vec.push_back(y_plane_markers);

  // horizontal plane markers
  visualization_msgs::msg::Marker hort_plane_marker;
  hort_plane_marker.pose.orientation.w = 1.0;
  hort_plane_marker.scale.x = 0.05;
  hort_plane_marker.scale.y = 0.05;
  hort_plane_marker.scale.z = 0.05;
  // plane_marker.points.resize(vert_planes.size());
  hort_plane_marker.header.frame_id = walls_layer_id;
  hort_plane_marker.header.stamp = stamp;
  hort_plane_marker.ns = "hort_planes";
  hort_plane_marker.id = hort_plane_markers.markers.size();
  hort_plane_marker.lifetime = duration_planes;
  hort_plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

  for (int i = 0; i < hort_plane_snapshot.size(); ++i) {
    for (size_t j = 0; j < hort_plane_snapshot[i].cloud_seg_map->size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = hort_plane_snapshot[i].cloud_seg_map->points[j].x;
      point.y = hort_plane_snapshot[i].cloud_seg_map->points[j].y;
      point.z = hort_plane_snapshot[i].cloud_seg_map->points[j].z;
      hort_plane_marker.points.push_back(point);
    }
    hort_plane_marker.color.r = 1;
    hort_plane_marker.color.g = 0.65;
    hort_plane_marker.color.a = 0.5;
  }
  hort_plane_markers.markers.push_back(hort_plane_marker);
  markers_vec.push_back(hort_plane_markers);

  rclcpp::Duration duration_room = rclcpp::Duration::from_seconds(5);
  for (auto& single_x_infinite_room : x_infinite_room_snapshot) {
    single_x_infinite_room.sub_infinite_room = false;
  }

  for (int i = 0; i < x_infinite_room_snapshot.size(); ++i) {
    if (x_infinite_room_snapshot[i].sub_infinite_room) continue;

    bool overlapped_infinite_room = false;
    float dist_room_x_corr = 100;
    for (const auto& room : room_snapshot) {
      if ((room.plane_x1_id == x_infinite_room_snapshot[i].plane1_id ||
           room.plane_x1_id == x_infinite_room_snapshot[i].plane2_id) &&
          (room.plane_x2_id == x_infinite_room_snapshot[i].plane1_id ||
           room.plane_x2_id == x_infinite_room_snapshot[i].plane2_id)) {
        overlapped_infinite_room = true;
        break;
      }
      dist_room_x_corr =
          sqrt(pow(room.node->estimate().translation()(0) -
                       x_infinite_room_snapshot[i].node->estimate().translation()(0),
                   2) +
               pow(room.node->estimate().translation()(1) -
                       x_infinite_room_snapshot[i].node->estimate().translation()(1),
                   2));
      if (dist_room_x_corr < 1.0) {
        overlapped_infinite_room = true;
        break;
      }
    }

    float dist_x_corr = 100;
    for (auto& current_x_infinite_room : x_infinite_room_snapshot) {
      if (current_x_infinite_room.id == x_infinite_room_snapshot[i].id) continue;

      dist_x_corr =
          sqrt(pow(current_x_infinite_room.node->estimate().translation()(0) -
                       x_infinite_room_snapshot[i].node->estimate().translation()(0),
                   2) +
               pow(current_x_infinite_room.node->estimate().translation()(1) -
                       x_infinite_room_snapshot[i].node->estimate().translation()(1),
                   2));
      if (dist_x_corr < 2.0) {
        current_x_infinite_room.sub_infinite_room = true;
        break;
      }
    }

    auto found_plane1 = std::find_if(
        x_plane_snapshot.begin(),
        x_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == x_infinite_room_snapshot[i].plane1_id);
    auto found_plane2 = std::find_if(
        x_plane_snapshot.begin(),
        x_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == x_infinite_room_snapshot[i].plane2_id);

    // fill in the line marker
    visualization_msgs::msg::Marker x_infinite_room_line_marker;
    x_infinite_room_line_marker.scale.x = 0.02;
    x_infinite_room_line_marker.pose.orientation.w = 1.0;
    if (!overlapped_infinite_room) {
      x_infinite_room_line_marker.ns =
          "infinite_room_x_lines " +
          std::to_string(x_infinite_room_snapshot[i].node->id());
      x_infinite_room_line_marker.header.frame_id = rooms_layer_id;
      x_infinite_room_line_marker.header.stamp = stamp;
      x_infinite_room_line_marker.id = x_infinite_room_edge_markers.markers.size() + 1;
      x_infinite_room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      x_infinite_room_line_marker.color.r = color_r;
      x_infinite_room_line_marker.color.g = color_g;
      x_infinite_room_line_marker.color.b = color_b;
      x_infinite_room_line_marker.color.a = 1.0;
      x_infinite_room_line_marker.lifetime = duration_room;
    } else {
      x_infinite_room_snapshot[i].id = -1;
      x_infinite_room_line_marker.ns =
          "overlapped_infinite_room_x_lines" +
          std::to_string(x_infinite_room_snapshot[i].node->id());
    }

    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = x_infinite_room_snapshot[i].node->estimate().translation()(0);
    p1.y = x_infinite_room_snapshot[i].node->estimate().translation()(1);
    p1.z = 0;

    float min_dist_plane1 = 100;
    for (int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_plane1).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_plane1).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_plane1).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_plane1) {
        min_dist_plane1 = norm;
        p2 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
    point2_stamped.header.frame_id = walls_layer_id;
    point2_stamped.point.x = p2.x;
    point2_stamped.point.y = p2.y;
    point2_stamped.point.z = p2.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point2_stamped,
                         point2_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);

    p2 = point2_stamped_transformed.point;
    x_infinite_room_line_marker.points.push_back(p1);
    x_infinite_room_line_marker.points.push_back(p2);

    float min_dist_plane2 = 100;
    for (int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_plane2).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_plane2).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_plane2).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_plane2) {
        min_dist_plane2 = norm;
        p3 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point3_stamped, point3_stamped_transformed;
    point3_stamped.header.frame_id = walls_layer_id;
    point3_stamped.point.x = p3.x;
    point3_stamped.point.y = p3.y;
    point3_stamped.point.z = p3.z;
    // convert point p3 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point3_stamped,
                         point3_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);

    p3 = point3_stamped_transformed.point;
    x_infinite_room_line_marker.points.push_back(p1);
    x_infinite_room_line_marker.points.push_back(p3);
    x_infinite_room_edge_markers.markers.push_back(x_infinite_room_line_marker);

    // x infinite_room cube
    visualization_msgs::msg::Marker infinite_room_pose_marker;
    infinite_room_pose_marker.scale.x = 0.5;
    infinite_room_pose_marker.scale.y = 0.5;
    infinite_room_pose_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    infinite_room_pose_marker.header.frame_id = rooms_layer_id;
    infinite_room_pose_marker.header.stamp = stamp;
    if (!overlapped_infinite_room) {
      infinite_room_pose_marker.ns =
          "x_infinite_room " + std::to_string(x_infinite_room_snapshot[i].node->id());
      infinite_room_pose_marker.id = x_infinite_room_markers.markers.size();
      infinite_room_pose_marker.type = visualization_msgs::msg::Marker::CUBE;
      infinite_room_pose_marker.color.r = 1;
      infinite_room_pose_marker.color.g = 0.64;
      infinite_room_pose_marker.color.a = 1;
      infinite_room_pose_marker.pose.position.x =
          x_infinite_room_snapshot[i].node->estimate().translation()(0);
      infinite_room_pose_marker.pose.position.y =
          x_infinite_room_snapshot[i].node->estimate().translation()(1);
      infinite_room_pose_marker.pose.position.z =
          x_infinite_room_snapshot[i].node->estimate().translation()(2);
      Eigen::Quaterniond quat(x_infinite_room_snapshot[i].node->estimate().linear());
      infinite_room_pose_marker.pose.orientation.x = quat.x();
      infinite_room_pose_marker.pose.orientation.y = quat.y();
      infinite_room_pose_marker.pose.orientation.z = quat.z();
      infinite_room_pose_marker.pose.orientation.w = quat.w();

      infinite_room_pose_marker.lifetime = duration_room;
      x_infinite_room_markers.markers.push_back(infinite_room_pose_marker);
      /* room clusters */
      int cluster_id = 0;
      for (auto& cluster : x_infinite_room_snapshot[i].cluster_array.markers) {
        cluster.header.frame_id = walls_layer_id;
        if (cluster_id == 0) cluster.ns = "x_infinite_vertex";
        if (cluster_id == 1) cluster.ns = "x_infinite_vertex_edges";
        cluster.id = x_infinite_room_markers.markers.size() + 1;
        x_infinite_room_markers.markers.push_back(cluster);
      }
    } else
      infinite_room_pose_marker.ns = "overlapped_x_infinite_room";
  }

  for (auto& single_y_infinite_room : y_infinite_room_snapshot) {
    single_y_infinite_room.sub_infinite_room = false;
  }

  for (int i = 0; i < y_infinite_room_snapshot.size(); ++i) {
    if (y_infinite_room_snapshot[i].sub_infinite_room) continue;

    bool overlapped_infinite_room = false;
    float dist_room_y_corr = 100;
    for (const auto& room : room_snapshot) {
      if ((room.plane_y1_id == y_infinite_room_snapshot[i].plane1_id ||
           room.plane_y1_id == y_infinite_room_snapshot[i].plane2_id) ||
          (room.plane_y2_id == y_infinite_room_snapshot[i].plane1_id ||
           room.plane_y2_id == y_infinite_room_snapshot[i].plane2_id)) {
        overlapped_infinite_room = true;
        break;
      }
      dist_room_y_corr =
          sqrt(pow(room.node->estimate().translation()(0) -
                       y_infinite_room_snapshot[i].node->estimate().translation()(0),
                   2) +
               pow(room.node->estimate().translation()(1) -
                       y_infinite_room_snapshot[i].node->estimate().translation()(1),
                   2));
      if (dist_room_y_corr < 1.0) {
        overlapped_infinite_room = true;
        break;
      }
    }

    float dist_y_corr = 100;
    for (auto& current_y_infinite_room : y_infinite_room_snapshot) {
      if (current_y_infinite_room.id == y_infinite_room_snapshot[i].id) continue;
      dist_y_corr =
          sqrt(pow(current_y_infinite_room.node->estimate().translation()(0) -
                       y_infinite_room_snapshot[i].node->estimate().translation()(0),
                   2) +
               pow(current_y_infinite_room.node->estimate().translation()(1) -
                       y_infinite_room_snapshot[i].node->estimate().translation()(1),
                   2));
      if (dist_y_corr < 2.0) {
        current_y_infinite_room.sub_infinite_room = true;
        break;
      }
    }

    auto found_plane1 = std::find_if(
        y_plane_snapshot.begin(),
        y_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == y_infinite_room_snapshot[i].plane1_id);
    auto found_plane2 = std::find_if(
        y_plane_snapshot.begin(),
        y_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == y_infinite_room_snapshot[i].plane2_id);

    // fill in the line marker
    visualization_msgs::msg::Marker y_infinite_room_line_marker;
    y_infinite_room_line_marker.scale.x = 0.02;
    y_infinite_room_line_marker.pose.orientation.w = 1.0;
    if (!overlapped_infinite_room) {
      y_infinite_room_line_marker.ns =
          "infinite_room_y_lines" +
          std::to_string(y_infinite_room_snapshot[i].node->id());
      y_infinite_room_line_marker.header.frame_id = rooms_layer_id;
      y_infinite_room_line_marker.header.stamp = stamp;
      y_infinite_room_line_marker.id = y_infinite_room_edge_markers.markers.size() + 1;
      y_infinite_room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      y_infinite_room_line_marker.color.r = color_r;
      y_infinite_room_line_marker.color.g = color_g;
      y_infinite_room_line_marker.color.b = color_b;
      y_infinite_room_line_marker.color.a = 1.0;
      y_infinite_room_line_marker.lifetime = duration_room;
    } else {
      y_infinite_room_snapshot[i].id = -1;
      y_infinite_room_line_marker.ns = "overlapped_infinite_room_y_lines";
    }

    geometry_msgs::msg::Point p1, p2, p3;
    p1.x = y_infinite_room_snapshot[i].node->estimate().translation()(0);
    p1.y = y_infinite_room_snapshot[i].node->estimate().translation()(1);
    p1.z = 0;

    float min_dist_plane1 = 100;
    for (int p = 0; p < (*found_plane1).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_plane1).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_plane1).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_plane1).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_plane1) {
        min_dist_plane1 = norm;
        p2 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
    point2_stamped.header.frame_id = walls_layer_id;
    point2_stamped.point.x = p2.x;
    point2_stamped.point.y = p2.y;
    point2_stamped.point.z = p2.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point2_stamped,
                         point2_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);

    p2 = point2_stamped_transformed.point;
    y_infinite_room_line_marker.points.push_back(p1);
    y_infinite_room_line_marker.points.push_back(p2);

    float min_dist_plane2 = 100;
    for (int p = 0; p < (*found_plane2).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_plane2).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_plane2).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_plane2).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_plane2) {
        min_dist_plane2 = norm;
        p3 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point3_stamped, point3_stamped_transformed;
    point3_stamped.header.frame_id = walls_layer_id;
    point3_stamped.point.x = p3.x;
    point3_stamped.point.y = p3.y;
    point3_stamped.point.z = p3.z;
    // convert point p3 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point3_stamped,
                         point3_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);

    p3 = point3_stamped_transformed.point;
    y_infinite_room_line_marker.points.push_back(p1);
    y_infinite_room_line_marker.points.push_back(p3);
    y_infinite_room_edge_markers.markers.push_back(y_infinite_room_line_marker);

    // y infinite_room cube
    visualization_msgs::msg::Marker infinite_room_pose_marker;
    infinite_room_pose_marker.pose.orientation.w = 1.0;
    infinite_room_pose_marker.scale.x = 0.5;
    infinite_room_pose_marker.scale.y = 0.5;
    infinite_room_pose_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    infinite_room_pose_marker.header.frame_id = rooms_layer_id;
    infinite_room_pose_marker.header.stamp = stamp;
    if (!overlapped_infinite_room) {
      infinite_room_pose_marker.ns =
          "y_infinite_room " + std::to_string(y_infinite_room_snapshot[i].node->id());
      infinite_room_pose_marker.id = y_infinite_room_markers.markers.size();
      infinite_room_pose_marker.type = visualization_msgs::msg::Marker::CUBE;
      infinite_room_pose_marker.color.r = 0.13;
      infinite_room_pose_marker.color.g = 0.54;
      infinite_room_pose_marker.color.b = 0.13;
      infinite_room_pose_marker.color.a = 1;
      infinite_room_pose_marker.pose.position.x =
          y_infinite_room_snapshot[i].node->estimate().translation()(0);
      infinite_room_pose_marker.pose.position.y =
          y_infinite_room_snapshot[i].node->estimate().translation()(1);
      infinite_room_pose_marker.pose.position.z =
          y_infinite_room_snapshot[i].node->estimate().translation()(2);
      Eigen::Quaterniond quat(y_infinite_room_snapshot[i].node->estimate().linear());
      infinite_room_pose_marker.pose.orientation.x = quat.x();
      infinite_room_pose_marker.pose.orientation.y = quat.y();
      infinite_room_pose_marker.pose.orientation.z = quat.z();
      infinite_room_pose_marker.pose.orientation.w = quat.w();
      infinite_room_pose_marker.lifetime = duration_room;
      y_infinite_room_markers.markers.push_back(infinite_room_pose_marker);
      /* room clusters */
      int cluster_id = 0;
      for (auto& cluster : y_infinite_room_snapshot[i].cluster_array.markers) {
        cluster.header.frame_id = walls_layer_id;
        if (cluster_id == 0) cluster.ns = "y_infinite_vertex";
        if (cluster_id == 1) cluster.ns = "y_infinite_vertex_edges";
        cluster.id = y_infinite_room_markers.markers.size() + 1;
        y_infinite_room_markers.markers.push_back(cluster);
      }
    } else
      infinite_room_pose_marker.ns = "overlapped_y_infinite_room";
  }
  markers_vec.push_back(x_infinite_room_edge_markers);
  markers_vec.push_back(y_infinite_room_edge_markers);
  markers_vec.push_back(x_infinite_room_markers);
  markers_vec.push_back(y_infinite_room_markers);
  // // room markers
  for (int i = 0; i < room_snapshot.size(); ++i) {
    room_snapshot[i].sub_room = false;
  }

  for (int i = 0; i < room_snapshot.size(); ++i) {
    if (room_snapshot[i].sub_room) continue;

    for (auto& room : room_snapshot) {
      if (room.id == room_snapshot[i].id) continue;
      float dist_room_room =
          sqrt(pow(room.node->estimate().translation()(0) -
                       room_snapshot[i].node->estimate().translation()(0),
                   2) +
               pow(room.node->estimate().translation()(1) -
                       room_snapshot[i].node->estimate().translation()(1),
                   2));
      if (dist_room_room < 2.0 && room.sub_room == false) {
        room.sub_room = true;
      }
    }

    //   // fill the pose marker
    visualization_msgs::msg::Marker room_marker;
    room_marker.scale.x = 0.5;
    room_marker.scale.y = 0.5;
    room_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    room_marker.header.frame_id = rooms_layer_id;
    room_marker.header.stamp = stamp;
    room_marker.ns = "room " + std::to_string(room_snapshot[i].node->id());
    room_marker.id = room_markers.markers.size();
    room_marker.type = visualization_msgs::msg::Marker::CUBE;
    room_marker.color.r = 1;
    room_marker.color.g = 0.07;
    room_marker.color.b = 0.57;
    room_marker.color.a = 1;

    room_marker.pose.position.x = room_snapshot[i].node->estimate().translation()(0);
    room_marker.pose.position.y = room_snapshot[i].node->estimate().translation()(1);
    room_marker.pose.position.z = room_snapshot[i].node->estimate().translation()(2);
    Eigen::Quaterniond quat(room_snapshot[i].node->estimate().linear());
    room_marker.pose.orientation.x = quat.x();
    room_marker.pose.orientation.y = quat.y();
    room_marker.pose.orientation.z = quat.z();
    room_marker.pose.orientation.w = quat.w();
    room_marker.lifetime = duration_room;
    room_markers.markers.push_back(room_marker);

    // fill in the line marker
    visualization_msgs::msg::Marker room_line_marker;
    room_line_marker.scale.x = 0.02;
    room_line_marker.pose.orientation.w = 1.0;
    room_line_marker.ns =
        "room_lines " + std::to_string(room_snapshot[i].node->id() + i);
    room_line_marker.header.frame_id = rooms_layer_id;
    room_line_marker.header.stamp = stamp;
    room_line_marker.id = room_edge_markers.markers.size() + 1;
    room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    room_line_marker.color.r = color_r;
    room_line_marker.color.g = color_g;
    room_line_marker.color.b = color_b;
    room_line_marker.color.a = 1.0;
    room_line_marker.lifetime = duration_room;
    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x = room_snapshot[i].node->estimate().translation()(0);
    p1.y = room_snapshot[i].node->estimate().translation()(1);
    p1.z = room_snapshot[i].node->estimate().translation()(2);

    auto found_planex1 = std::find_if(
        x_plane_snapshot.begin(),
        x_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_x1_id);
    auto found_planex2 = std::find_if(
        x_plane_snapshot.begin(),
        x_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_x2_id);
    auto found_planey1 = std::find_if(
        y_plane_snapshot.begin(),
        y_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_y1_id);
    auto found_planey2 = std::find_if(
        y_plane_snapshot.begin(),
        y_plane_snapshot.end(),
        boost::bind(&VerticalPlanes::id, _1) == room_snapshot[i].plane_y2_id);

    float min_dist_x1 = 100;
    for (int p = 0; p < (*found_planex1).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_planex1).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_planex1).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_planex1).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_x1) {
        min_dist_x1 = norm;
        p2 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
    point2_stamped.header.frame_id = walls_layer_id;
    point2_stamped.point.x = p2.x;
    point2_stamped.point.y = p2.y;
    point2_stamped.point.z = p2.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point2_stamped,
                         point2_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);
    p2 = point2_stamped_transformed.point;
    room_line_marker.points.push_back(p1);
    room_line_marker.points.push_back(p2);

    float min_dist_x2 = 100;
    for (int p = 0; p < (*found_planex2).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_planex2).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_planex2).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_planex2).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_x2) {
        min_dist_x2 = norm;
        p3 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point3_stamped, point3_stamped_transformed;
    point3_stamped.header.frame_id = walls_layer_id;
    point3_stamped.point.x = p3.x;
    point3_stamped.point.y = p3.y;
    point3_stamped.point.z = p3.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point3_stamped,
                         point3_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);
    p3 = point3_stamped_transformed.point;
    room_line_marker.points.push_back(p1);
    room_line_marker.points.push_back(p3);

    float min_dist_y1 = 100;
    for (int p = 0; p < (*found_planey1).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_planey1).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_planey1).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_planey1).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_y1) {
        min_dist_y1 = norm;
        p4 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point4_stamped, point4_stamped_transformed;
    point4_stamped.header.frame_id = walls_layer_id;
    point4_stamped.point.x = p4.x;
    point4_stamped.point.y = p4.y;
    point4_stamped.point.z = p4.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point4_stamped,
                         point4_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);
    p4 = point4_stamped_transformed.point;
    room_line_marker.points.push_back(p1);
    room_line_marker.points.push_back(p4);

    float min_dist_y2 = 100;
    for (int p = 0; p < (*found_planey2).cloud_seg_map->points.size(); ++p) {
      geometry_msgs::msg::Point p_tmp;
      p_tmp.x = (*found_planey2).cloud_seg_map->points[p].x;
      p_tmp.y = (*found_planey2).cloud_seg_map->points[p].y;
      p_tmp.z = (*found_planey2).cloud_seg_map->points[p].z;

      float norm =
          std::sqrt(std::pow((p1.x - p_tmp.x), 2) + std::pow((p1.y - p_tmp.y), 2) +
                    std::pow((p1.z - p_tmp.z), 2));

      if (norm < min_dist_y2) {
        min_dist_y2 = norm;
        p5 = p_tmp;
      }
    }

    geometry_msgs::msg::PointStamped point5_stamped, point5_stamped_transformed;
    point5_stamped.header.frame_id = walls_layer_id;
    point5_stamped.point.x = p5.x;
    point5_stamped.point.y = p5.y;
    point5_stamped.point.z = p5.z;

    // convert point p2 to rooms_layer_id currently it is map_frame_id
    tf_buffer->transform(point5_stamped,
                         point5_stamped_transformed,
                         rooms_layer_id,
                         tf2::TimePointZero,
                         walls_layer_id);
    p5 = point5_stamped_transformed.point;
    room_line_marker.points.push_back(p1);
    room_line_marker.points.push_back(p5);

    room_edge_markers.markers.push_back(room_line_marker);
  }
  markers_vec.push_back(room_markers);
  markers_vec.push_back(room_edge_markers);

  //   /* room clusters */
  //   int cluster_id = 0;
  //   for (auto& cluster : room_snapshot[i].cluster_array.markers) {
  //     cluster.header.frame_id = walls_layer_id;
  //     if (cluster_id == 0) cluster.ns = "room_vertex";
  //     if (cluster_id == 1) cluster.ns = "room_edges";
  //     cluster.id = markers.markers.size() + 1;
  //     markers.markers.push_back(cluster);
  //   }
  // }

  rclcpp::Duration duration_floor = rclcpp::Duration::from_seconds(5);
  for (const auto& floor : floors_vec) {
    if (floor.id != -1) {
      visualization_msgs::msg::Marker floor_marker;
      floor_marker.pose.orientation.w = 1.0;
      floor_marker.scale.x = 0.5;
      floor_marker.scale.y = 0.5;
      floor_marker.scale.z = 0.5;
      // plane_marker.points.resize(vert_planes.size());
      floor_marker.header.frame_id = floors_layer_id;
      floor_marker.header.stamp = stamp;
      floor_marker.ns = "floor " + std::to_string(floor.node->id());
      floor_marker.id = floor_markers.markers.size();
      floor_marker.type = visualization_msgs::msg::Marker::CUBE;
      floor_marker.color.r = 0.49;
      floor_marker.color.g = 0;
      floor_marker.color.b = 1;
      floor_marker.color.a = 1;
      floor_marker.lifetime = duration_floor;

      floor_marker.pose.position.x = floor.node->estimate().translation()(0);
      floor_marker.pose.position.y = floor.node->estimate().translation()(1);
      floor_marker.pose.position.z = floor.node->estimate().translation()(2);

      // create line markers between floor and rooms/infinite_rooms
      visualization_msgs::msg::Marker floor_line_marker;
      floor_line_marker.scale.x = 0.02;
      floor_line_marker.pose.orientation.w = 1.0;

      floor_line_marker.header.frame_id = floors_layer_id;
      floor_line_marker.header.stamp = stamp;
      floor_line_marker.id = floor_edge_markers.markers.size() + 1;
      floor_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      floor_line_marker.color.r = color_r;
      floor_line_marker.color.g = color_g;
      floor_line_marker.color.b = color_b;
      floor_line_marker.color.a = 1.0;
      floor_line_marker.lifetime = duration_floor;

      for (const auto& room : room_snapshot) {
        if (room.sub_room) continue;
        geometry_msgs::msg::Point p1, p2;
        floor_line_marker.ns = "floor_line " + std::to_string(room.node->id());
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = room.node->estimate().translation()(0);
        p2.y = room.node->estimate().translation()(1);
        p2.z = room.node->estimate().translation()(2);

        geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
        point2_stamped.header.frame_id = rooms_layer_id;
        point2_stamped.point.x = p2.x;
        point2_stamped.point.y = p2.y;
        point2_stamped.point.z = p2.z;

        // convert point p2 to rooms_layer_id currently it is map_frame_id
        tf_buffer->transform(point2_stamped,
                             point2_stamped_transformed,
                             floors_layer_id,
                             tf2::TimePointZero,
                             rooms_layer_id);
        p2 = point2_stamped_transformed.point;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& x_infinite_room : x_infinite_room_snapshot) {
        if (x_infinite_room.id == -1 || x_infinite_room.sub_infinite_room) continue;
        geometry_msgs::msg::Point p1, p2;
        floor_line_marker.ns =
            "floor_line " + std::to_string(x_infinite_room.node->id());
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = x_infinite_room.node->estimate().translation()(0);
        p2.y = x_infinite_room.node->estimate().translation()(1);
        p2.z = x_infinite_room.node->estimate().translation()(2);

        geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
        point2_stamped.header.frame_id = rooms_layer_id;
        point2_stamped.point.x = p2.x;
        point2_stamped.point.y = p2.y;
        point2_stamped.point.z = p2.z;

        // convert point p2 to rooms_layer_id currently it is map_frame_id
        tf_buffer->transform(point2_stamped,
                             point2_stamped_transformed,
                             floors_layer_id,
                             tf2::TimePointZero,
                             rooms_layer_id);
        p2 = point2_stamped_transformed.point;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      for (const auto& y_infinite_room : y_infinite_room_snapshot) {
        if (y_infinite_room.id == -1 || y_infinite_room.sub_infinite_room) continue;
        geometry_msgs::msg::Point p1, p2;
        floor_line_marker.ns =
            "floor_line " + std::to_string(y_infinite_room.node->id());
        p1.x = floor_marker.pose.position.x;
        p1.y = floor_marker.pose.position.y;
        p1.z = floor_marker.pose.position.z;
        p2.x = y_infinite_room.node->estimate().translation()(0);
        p2.y = y_infinite_room.node->estimate().translation()(1);
        p2.z = y_infinite_room.node->estimate().translation()(2);

        geometry_msgs::msg::PointStamped point2_stamped, point2_stamped_transformed;
        point2_stamped.header.frame_id = rooms_layer_id;
        point2_stamped.point.x = p2.x;
        point2_stamped.point.y = p2.y;
        point2_stamped.point.z = p2.z;

        // convert point p2 to rooms_layer_id currently it is map_frame_id
        tf_buffer->transform(point2_stamped,
                             point2_stamped_transformed,
                             floors_layer_id,
                             tf2::TimePointZero,
                             rooms_layer_id);
        p2 = point2_stamped_transformed.point;
        floor_line_marker.points.push_back(p1);
        floor_line_marker.points.push_back(p2);
      }
      floor_markers.markers.push_back(floor_marker);
      floor_edge_markers.markers.push_back(floor_line_marker);
    }
  }
  markers_vec.push_back(floor_markers);
  markers_vec.push_back(floor_edge_markers);

  return markers_vec;
}
std::vector<visualization_msgs::msg::MarkerArray>
GraphVisualizer::create_prior_marker_array(
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
  std::vector<visualization_msgs::msg::MarkerArray> markers_vec;
  visualization_msgs::msg::MarkerArray prior_markers;
  double plane_h = 15;
  double wall_vertex_h = 18;
  double prior_room_h = 22;
  double deviation_h = 16;
  prior_markers.markers.clear();

  for (int i = 0; i < x_vert_planes_prior.size(); i++) {  // walls_x_coord.size()
    double r, g, b;
    visualization_msgs::msg::Marker wall_visual_marker;
    if (!got_trans_prior2map_) {
      wall_visual_marker.header.frame_id = "prior_map";
    } else {
      wall_visual_marker.header.frame_id = map_frame_id;
    }
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

    wall_visual_marker.color.r = x_vert_planes_prior[i].color[0] / 255;
    wall_visual_marker.color.g = x_vert_planes_prior[i].color[1] / 255;
    wall_visual_marker.color.b = x_vert_planes_prior[i].color[2] / 255;
    wall_visual_marker.color.a = 0.3;
    prior_markers.markers.push_back(wall_visual_marker);
  }

  for (int i = 0; i < y_vert_planes_prior.size(); i++) {  // walls_x_coord.size()

    double r, g, b;
    visualization_msgs::msg::Marker wall_visual_marker;
    if (!got_trans_prior2map_) {
      wall_visual_marker.header.frame_id = "prior_map";
    } else {
      wall_visual_marker.header.frame_id = map_frame_id;
    }
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

    wall_visual_marker.color.r = y_vert_planes_prior[i].color[0] / 255;
    wall_visual_marker.color.g = y_vert_planes_prior[i].color[1] / 255;
    wall_visual_marker.color.b = y_vert_planes_prior[i].color[2] / 255;
    wall_visual_marker.color.a = 0.3;
    prior_markers.markers.push_back(wall_visual_marker);
  }
  markers_vec.push_back(prior_markers);

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
      wall_center_marker.scale.x = 0.2;
      wall_center_marker.scale.y = 0.2;
      wall_center_marker.scale.z = 0.2;
      wall_center_marker.pose.position.x = wall_center.x();
      wall_center_marker.pose.position.y = wall_center.y();
      wall_center_marker.pose.position.z = wall_vertex_h;
      wall_center_marker.pose.orientation.x = 0.0;
      wall_center_marker.pose.orientation.y = 0.0;
      wall_center_marker.pose.orientation.z = 0.0;
      wall_center_marker.pose.orientation.w = 1.0;
      prior_markers.markers.push_back(wall_center_marker);

      // wall surface plane edge markers
      visualization_msgs::msg::Marker wall_edge_plane_marker;
      if (!got_trans_prior2map_) {
        wall_edge_plane_marker.header.frame_id = "prior_map";
      } else {
        wall_edge_plane_marker.header.frame_id = map_frame_id;
      }
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
      prior_markers.markers.push_back(wall_edge_plane_marker);
      visualization_msgs::msg::Marker wall_edge_plane_marker_2;
      if (!got_trans_prior2map_) {
        wall_edge_plane_marker_2.header.frame_id = "prior_map";
      } else {
        wall_edge_plane_marker_2.header.frame_id = map_frame_id;
      }
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
      prior_markers.markers.push_back(wall_edge_plane_marker_2);
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
      // std::cout << "Deviation between : " << v2->id() << "  and: " << v3->id()
      //           << std::endl;
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
          visualization_msgs::msg::Marker wall_deviation_marker;
          wall_deviation_marker.header.frame_id = map_frame_id;
          wall_deviation_marker.header.stamp = stamp;
          wall_deviation_marker.ns = "deviations";
          wall_deviation_marker.id = prior_markers.markers.size() + i;
          wall_deviation_marker.type = visualization_msgs::msg::Marker::SPHERE;
          wall_deviation_marker.action = visualization_msgs::msg::Marker::ADD;
          wall_deviation_marker.scale.x = 0.3;
          wall_deviation_marker.scale.y = 0.3;
          wall_deviation_marker.scale.z = 0.3;
          wall_deviation_marker.color.r = 0.0;
          wall_deviation_marker.color.g = 0.0;
          wall_deviation_marker.color.b = 0.0;
          wall_deviation_marker.color.a = 1.0;

          Eigen::Vector3d translation = dev_pose.block<3, 1>(0, 3);
          Eigen::Matrix3d rotation_matrix = dev_pose.block<3, 3>(0, 0);
          Eigen::Quaterniond quaternion(rotation_matrix);
          quaternion.normalize();
          wall_deviation_marker.pose.position.x = translation.x();
          p2.x = wall_deviation_marker.pose.position.x;
          wall_deviation_marker.pose.position.y = translation.y();
          p2.y = wall_deviation_marker.pose.position.y;
          p2.z = plane_h;
          wall_deviation_marker.pose.position.z = wall_vertex_h;
          wall_deviation_marker.pose.orientation.x = quaternion.x();
          wall_deviation_marker.pose.orientation.y = quaternion.y();
          wall_deviation_marker.pose.orientation.z = quaternion.z();
          prior_markers.markers.push_back(wall_deviation_marker);
          Eigen::Isometry3d a_graph_wall_pose;
          auto found_a_graph_plane = x_vert_planes_prior.begin();
          found_a_graph_plane =
              std::find_if(x_vert_planes_prior.begin(),
                           x_vert_planes_prior.end(),
                           boost::bind(&s_graphs::VerticalPlanes::id, _1) == v2->id());

          // give the start and end points of the a graph plan calculate the closest
          // point to the s_graph plane
          Eigen::Vector2d a_graph_plane_start_point;
          a_graph_plane_start_point[0] = (*found_a_graph_plane).start_point.x();
          a_graph_plane_start_point[1] = (*found_a_graph_plane).start_point.y();
          Eigen::Vector2d a_graph_direction(
              abs((*found_a_graph_plane).plane_node->estimate().coeffs()(1)),
              abs((*found_a_graph_plane).plane_node->estimate().coeffs()(0)));
          std::vector<Eigen::Vector2d> a_graph_plane_segments =
              divide_plane_into_segments(a_graph_plane_start_point,
                                         (*found_a_graph_plane).length,
                                         a_graph_direction,
                                         100);
          Eigen::Vector2d s_graph_point;
          s_graph_point[0] = p2.x;
          s_graph_point[1] = p2.y;
          Eigen::Vector2d closest_point =
              find_closest_point(s_graph_point, a_graph_plane_segments);
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
          p3.x = closest_point.x();
          p3.y = closest_point.y();
          p3.z = plane_h;

          visualization_msgs::msg::Marker deviation_wall_edge_marker;
          deviation_wall_edge_marker.header.frame_id = map_frame_id;
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

          visualization_msgs::msg::Marker deviation_wall_edge_marker2;
          deviation_wall_edge_marker2.header.frame_id = map_frame_id;
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
          visualization_msgs::msg::Marker wall_deviation_marker;
          wall_deviation_marker.header.frame_id = map_frame_id;
          wall_deviation_marker.header.stamp = stamp;
          wall_deviation_marker.ns = "deviations";
          wall_deviation_marker.id = prior_markers.markers.size() + i;
          wall_deviation_marker.type = visualization_msgs::msg::Marker::SPHERE;
          wall_deviation_marker.action = visualization_msgs::msg::Marker::ADD;
          wall_deviation_marker.scale.x = 0.3;
          wall_deviation_marker.scale.y = 0.3;
          wall_deviation_marker.scale.z = 0.3;
          wall_deviation_marker.color.r = 0.0;
          wall_deviation_marker.color.g = 0.0;
          wall_deviation_marker.color.b = 0.0;
          wall_deviation_marker.color.a = 0.7;

          Eigen::Vector3d translation = dev_pose.block<3, 1>(0, 3);
          Eigen::Matrix3d rotation_matrix = dev_pose.block<3, 3>(0, 0);
          Eigen::Quaterniond quaternion(rotation_matrix);
          quaternion.normalize();
          wall_deviation_marker.pose.position.x = translation.x();
          p2.x = wall_deviation_marker.pose.position.x;
          wall_deviation_marker.pose.position.y = translation.y();
          p2.y = wall_deviation_marker.pose.position.y;
          wall_deviation_marker.pose.position.z = wall_vertex_h;
          p2.z = plane_h;
          wall_deviation_marker.pose.orientation.x = quaternion.x();
          wall_deviation_marker.pose.orientation.y = quaternion.y();
          wall_deviation_marker.pose.orientation.z = quaternion.z();
          wall_deviation_marker.pose.orientation.w = quaternion.w();
          prior_markers.markers.push_back(wall_deviation_marker);

          Eigen::Isometry3d a_graph_wall_pose = Eigen::Isometry3d::Identity();
          auto found_a_graph_plane = y_vert_planes_prior.begin();
          found_a_graph_plane =
              std::find_if(y_vert_planes_prior.begin(),
                           y_vert_planes_prior.end(),
                           boost::bind(&s_graphs::VerticalPlanes::id, _1) == v2->id());
          Eigen::Vector2d a_graph_plane_start_point;
          a_graph_plane_start_point[0] = (*found_a_graph_plane).start_point.x();
          a_graph_plane_start_point[1] = (*found_a_graph_plane).start_point.y();
          Eigen::Vector2d a_graph_direction(
              abs((*found_a_graph_plane).plane_node->estimate().coeffs()(1)),
              abs((*found_a_graph_plane).plane_node->estimate().coeffs()(0)));
          std::vector<Eigen::Vector2d> a_graph_plane_segments =
              divide_plane_into_segments(a_graph_plane_start_point,
                                         (*found_a_graph_plane).length,
                                         a_graph_direction,
                                         100);
          Eigen::Vector2d s_graph_point;
          s_graph_point[0] = p2.x;
          s_graph_point[1] = p2.y;
          Eigen::Vector2d closest_point =
              find_closest_point(s_graph_point, a_graph_plane_segments);
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
          p3.x = closest_point.x();
          p3.y = closest_point.y();
          p3.z = plane_h;
          visualization_msgs::msg::Marker deviation_wall_edge_marker;
          deviation_wall_edge_marker.header.frame_id = map_frame_id;
          deviation_wall_edge_marker.header.stamp = stamp;
          deviation_wall_edge_marker.ns = "wall_deviation_edges";
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

          visualization_msgs::msg::Marker deviation_wall_edge_marker2;
          deviation_wall_edge_marker2.header.frame_id = map_frame_id;
          deviation_wall_edge_marker2.header.stamp = stamp;
          deviation_wall_edge_marker2.ns = "wall_deviation_edges";
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
  markers_vec.push_back(prior_markers);

  auto room_dev_edge_iterator = local_graph->edges().begin();
  for (int i = 0; room_dev_edge_iterator != local_graph->edges().end();
       room_dev_edge_iterator++, i++) {
    g2o::HyperGraph::Edge* edge = *room_dev_edge_iterator;
    g2o::EdgeSE3RoomRoom* edge_room_dev = dynamic_cast<g2o::EdgeSE3RoomRoom*>(edge);
    if (edge_room_dev) {
      g2o::VertexDeviation* v1 =
          dynamic_cast<g2o::VertexDeviation*>(edge_room_dev->vertices()[0]);
      g2o::VertexRoom* v2 =
          dynamic_cast<g2o::VertexRoom*>(edge_room_dev->vertices()[1]);
      g2o::VertexRoom* v3 =
          dynamic_cast<g2o::VertexRoom*>(edge_room_dev->vertices()[2]);

      visualization_msgs::msg::Marker room_deviation_marker;
      room_deviation_marker.header.frame_id = map_frame_id;
      room_deviation_marker.header.stamp = stamp;
      room_deviation_marker.ns = "room_deviation";
      room_deviation_marker.id = prior_markers.markers.size() + i;
      room_deviation_marker.type = visualization_msgs::msg::Marker::CUBE;
      room_deviation_marker.action = visualization_msgs::msg::Marker::ADD;
      room_deviation_marker.scale.x = 0.3;
      room_deviation_marker.scale.y = 0.3;
      room_deviation_marker.scale.z = 0.3;
      room_deviation_marker.color.r = 0.0;
      room_deviation_marker.color.g = 0.0;
      room_deviation_marker.color.b = 0.0;
      room_deviation_marker.color.a = 0.7;

      auto found_a_graph_room = rooms_vec_prior.begin();
      found_a_graph_room =
          std::find_if(rooms_vec_prior.begin(),
                       rooms_vec_prior.end(),
                       boost::bind(&s_graphs::Rooms::id, _1) == v2->id());
      Eigen::Matrix4d a_graph_room_pose = Eigen::Matrix4d::Identity();
      Eigen::Vector3d a_graph_room_translation =
          (*found_a_graph_room).node->estimate().matrix().block<3, 1>(0, 3);
      Eigen::Matrix3d a_graph_room_rot_mat =
          (*found_a_graph_room).node->estimate().matrix().block<3, 3>(0, 0);
      Eigen::Quaterniond a_graph_room_quat(a_graph_room_rot_mat);

      room_deviation_marker.pose.position.x = a_graph_room_translation.x();
      room_deviation_marker.pose.position.y = a_graph_room_translation.y();
      room_deviation_marker.pose.position.z = prior_room_h + 1;
      room_deviation_marker.pose.orientation.x = a_graph_room_quat.x();
      room_deviation_marker.pose.orientation.y = a_graph_room_quat.y();
      room_deviation_marker.pose.orientation.z = a_graph_room_quat.z();
      room_deviation_marker.pose.orientation.w = a_graph_room_quat.w();

      prior_markers.markers.push_back(room_deviation_marker);
      geometry_msgs::msg::Point p1, p2, p3;
      p1.x = room_deviation_marker.pose.position.x;
      p1.y = room_deviation_marker.pose.position.y;
      p1.z = room_deviation_marker.pose.position.z;
      p2.x = room_deviation_marker.pose.position.x;
      p2.y = room_deviation_marker.pose.position.y;
      p2.z = prior_room_h;

      visualization_msgs::msg::Marker room_deviation_edge_marker_1;
      room_deviation_edge_marker_1.header.frame_id = map_frame_id;
      room_deviation_edge_marker_1.header.stamp = stamp;
      room_deviation_edge_marker_1.ns = "room_deviation_edges";
      room_deviation_edge_marker_1.id = prior_markers.markers.size();
      room_deviation_edge_marker_1.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_deviation_edge_marker_1.pose.orientation.w = 1.0;
      room_deviation_edge_marker_1.scale.x = 0.03;
      room_deviation_edge_marker_1.scale.x = 0.02;
      room_deviation_edge_marker_1.color.r = 0.0;
      room_deviation_edge_marker_1.color.g = 0.0;
      room_deviation_edge_marker_1.color.b = 0.0;
      room_deviation_edge_marker_1.color.a = 1.0;
      room_deviation_edge_marker_1.points.push_back(p1);
      room_deviation_edge_marker_1.points.push_back(p2);

      prior_markers.markers.push_back(room_deviation_edge_marker_1);

      auto found_s_graph_room = rooms_vec.begin();
      found_s_graph_room =
          std::find_if(rooms_vec.begin(),
                       rooms_vec.end(),
                       boost::bind(&s_graphs::Rooms::id, _1) == v3->id());
      Eigen::Matrix4d s_graph_room_pose = Eigen::Matrix4d::Identity();
      Eigen::Vector3d s_graph_room_translation =
          (*found_s_graph_room).node->estimate().matrix().block<3, 1>(0, 3);
      Eigen::Matrix3d s_graph_room_rot_mat =
          (*found_s_graph_room).node->estimate().matrix().block<3, 3>(0, 0);
      Eigen::Quaterniond s_graph_room_quat(s_graph_room_rot_mat);

      p3.x = s_graph_room_translation.x();
      p3.y = s_graph_room_translation.y();
      p3.z = prior_room_h;

      visualization_msgs::msg::Marker room_deviation_edge_marker_2;
      room_deviation_edge_marker_2.header.frame_id = map_frame_id;
      room_deviation_edge_marker_2.header.stamp = stamp;
      room_deviation_edge_marker_2.ns = "room_deviation_edges";
      room_deviation_edge_marker_2.id = prior_markers.markers.size();
      room_deviation_edge_marker_2.type = visualization_msgs::msg::Marker::LINE_LIST;
      room_deviation_edge_marker_2.pose.orientation.w = 1.0;
      room_deviation_edge_marker_2.scale.x = 0.03;
      room_deviation_edge_marker_2.scale.x = 0.02;
      room_deviation_edge_marker_2.color.r = 0.0;
      room_deviation_edge_marker_2.color.g = 0.0;
      room_deviation_edge_marker_2.color.b = 0.0;
      room_deviation_edge_marker_2.color.a = 1.0;
      room_deviation_edge_marker_2.points.push_back(p1);
      room_deviation_edge_marker_2.points.push_back(p3);

      prior_markers.markers.push_back(room_deviation_edge_marker_2);
    }
  }

  for (int i = 0; i < rooms_vec_prior.size(); i++) {  // walls_x_coord.size()
    double r, g, b;
    visualization_msgs::msg::Marker prior_room_marker;
    if (!got_trans_prior2map_) {
      prior_room_marker.header.frame_id = "prior_map";
    } else {
      prior_room_marker.header.frame_id = map_frame_id;
    }
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

    prior_room_marker.scale.y = 0.5;
    prior_room_marker.scale.x = 0.5;
    prior_room_marker.scale.z = 0.5;
    prior_room_marker.color.r = 221 / 255.0;
    prior_room_marker.color.g = 110 / 255.0;
    prior_room_marker.color.b = 230 / 255.0;
    prior_room_marker.color.a = 1.0;
    prior_markers.markers.push_back(prior_room_marker);

    // Edge plane 1
    geometry_msgs::msg::Point point1, point2, point3, point4, point5;
    point1.x = room_pose(0, 3);
    point1.y = room_pose(1, 3);
    point1.z = prior_room_h;
    visualization_msgs::msg::Marker room_edge_plane_marker1;
    if (!got_trans_prior2map_) {
      room_edge_plane_marker1.header.frame_id = "prior_map";
    } else {
      room_edge_plane_marker1.header.frame_id = map_frame_id;
    }
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
    room_edge_plane_marker1.scale.x = 0.02;
    room_edge_plane_marker1.color.r = 0.0;
    room_edge_plane_marker1.color.g = 0.0;
    room_edge_plane_marker1.color.b = 0.0;
    room_edge_plane_marker1.color.a = 1.0;
    room_edge_plane_marker1.points.push_back(point1);
    room_edge_plane_marker1.points.push_back(point2);
    prior_markers.markers.push_back(room_edge_plane_marker1);

    visualization_msgs::msg::Marker room_edge_plane_marker2;
    if (!got_trans_prior2map_) {
      room_edge_plane_marker2.header.frame_id = "prior_map";
    } else {
      room_edge_plane_marker2.header.frame_id = map_frame_id;
    }
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
    room_edge_plane_marker2.scale.x = 0.02;
    room_edge_plane_marker2.color.r = 0.0;
    room_edge_plane_marker2.color.g = 0.0;
    room_edge_plane_marker2.color.b = 0.0;
    room_edge_plane_marker2.color.a = 1.0;
    room_edge_plane_marker2.points.push_back(point1);
    room_edge_plane_marker2.points.push_back(point3);
    prior_markers.markers.push_back(room_edge_plane_marker2);

    visualization_msgs::msg::Marker room_edge_plane_marker3;
    if (!got_trans_prior2map_) {
      room_edge_plane_marker3.header.frame_id = "prior_map";
    } else {
      room_edge_plane_marker3.header.frame_id = map_frame_id;
    }
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
    room_edge_plane_marker3.scale.x = 0.02;
    room_edge_plane_marker3.color.r = 0.0;
    room_edge_plane_marker3.color.g = 0.0;
    room_edge_plane_marker3.color.b = 0.0;
    room_edge_plane_marker3.color.a = 1.0;
    room_edge_plane_marker3.points.push_back(point1);
    room_edge_plane_marker3.points.push_back(point4);
    prior_markers.markers.push_back(room_edge_plane_marker3);

    visualization_msgs::msg::Marker room_edge_plane_marker4;
    if (!got_trans_prior2map_) {
      room_edge_plane_marker4.header.frame_id = "prior_map";
    } else {
      room_edge_plane_marker4.header.frame_id = map_frame_id;
    }
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
    room_edge_plane_marker4.scale.x = 0.02;
    room_edge_plane_marker4.color.r = 0.0;
    room_edge_plane_marker4.color.g = 0.0;
    room_edge_plane_marker4.color.b = 0.0;
    room_edge_plane_marker4.color.a = 1.0;
    room_edge_plane_marker4.points.push_back(point1);
    room_edge_plane_marker4.points.push_back(point5);
    prior_markers.markers.push_back(room_edge_plane_marker4);
  }
  markers_vec.push_back(prior_markers);

  for (int i = 0; i < doorways_vec_prior.size(); i++) {  // walls_x_coord.size()

    double r, g, b, door_marker_size;
    door_marker_size = 1.0;

    visualization_msgs::msg::Marker prior_door_marker;
    if (!got_trans_prior2map_) {
      prior_door_marker.header.frame_id = "prior_map";
    } else {
      prior_door_marker.header.frame_id = map_frame_id;
    }
    prior_door_marker.header.stamp = stamp;
    prior_door_marker.ns = "prior_doorways";
    prior_door_marker.id = prior_markers.markers.size() + i;
    prior_door_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    prior_door_marker.mesh_resource = "package://is_graphs/meshes/door.dae";
    Eigen::Vector3d door_pose = doorways_vec_prior[i].door_pos_w;
    prior_door_marker.pose.position.x = door_pose.x();
    prior_door_marker.pose.position.y = door_pose.y();
    prior_door_marker.pose.position.z = prior_room_h - door_marker_size;
    prior_door_marker.pose.orientation.x = 0;
    prior_door_marker.pose.orientation.y = 0;
    prior_door_marker.pose.orientation.z = 0.0;
    prior_door_marker.pose.orientation.w = 1;

    prior_door_marker.scale.y = 0.08;
    prior_door_marker.scale.x = 0.5;
    prior_door_marker.scale.z = 0.3;
    prior_door_marker.color.r = 193 / 255.0;
    prior_door_marker.color.g = 154 / 255.0;
    prior_door_marker.color.b = 107 / 255.0;
    prior_door_marker.color.a = 1.0;
    prior_markers.markers.push_back(prior_door_marker);

    geometry_msgs::msg::Point point1, point2, point3;
    point1.x = door_pose.x();
    point1.y = door_pose.y();
    point1.z = (prior_room_h - door_marker_size) + 0.5;
    visualization_msgs::msg::Marker door_edge_plane_marker1;
    if (!got_trans_prior2map_) {
      door_edge_plane_marker1.header.frame_id = "prior_map";
    } else {
      door_edge_plane_marker1.header.frame_id = map_frame_id;
    }
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
    if (!got_trans_prior2map_) {
      door_edge_plane_marker2.header.frame_id = "prior_map";
    } else {
      door_edge_plane_marker2.header.frame_id = map_frame_id;
    }
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
  markers_vec.push_back(prior_markers);

  return markers_vec;
}

Eigen::Isometry3d GraphVisualizer::compute_plane_pose(const VerticalPlanes& plane,
                                                      pcl::PointXYZRGBNormal& p_min,
                                                      pcl::PointXYZRGBNormal& p_max) {
  double length = pcl::getMaxSegment(*plane.cloud_seg_map, p_min, p_max);

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
Eigen::Vector2d GraphVisualizer::calculate_end_point(const Eigen::Vector2d& start_point,
                                                     const Eigen::Vector2d& direction,
                                                     double length) {
  return start_point + length * direction;
}
std::vector<Eigen::Vector2d> GraphVisualizer::divide_plane_into_segments(
    Eigen::Vector2d start_point,
    double length,
    Eigen::Vector2d& direction,
    int segments) {
  std::vector<Eigen::Vector2d> segment_points;
  Eigen::Vector2d end_point = calculate_end_point(start_point, direction, length);
  double segmentLength = length / segments;

  for (int i = 0; i <= segments; ++i) {
    double currentLength = i * segmentLength;
    Eigen::Vector2d currentPoint =
        calculate_end_point(start_point, direction, currentLength);
    segment_points.push_back(currentPoint);
  }

  return segment_points;
}

double GraphVisualizer::calculate_distance(const Eigen::Vector2d& point1,
                                           const Eigen::Vector2d& point2) {
  return sqrt(std::pow(point1(0) - point2(0), 2) + std::pow(point1(1) - point2(1), 2));
}

// Function to find losest point to a given point
Eigen::Vector2d GraphVisualizer::find_closest_point(
    const Eigen::Vector2d& given_point,
    const std::vector<Eigen::Vector2d>& points) {
  if (points.empty()) {
    throw std::invalid_argument("Points vector is empty.");
  }

  Eigen::Vector2d closest_point = points[0];
  double min_distance = calculate_distance(given_point, closest_point);

  for (const auto& point : points) {
    double current_distance = calculate_distance(given_point, point);
    if (current_distance < min_distance) {
      closest_point = point;
      min_distance = current_distance;
    }
  }

  return closest_point;
}
}  // namespace s_graphs
