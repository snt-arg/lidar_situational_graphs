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
visualization_msgs::msg::MarkerArray GraphVisualizer::create_marker_array(
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
  visualization_msgs::msg::MarkerArray markers;
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

  visualization_msgs::msg::Marker traj_marker;
  traj_marker.header.frame_id = keyframes_layer_id;
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = markers.markers.size();
  traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

  visualization_msgs::msg::Marker imu_marker;
  imu_marker.header = traj_marker.header;
  imu_marker.ns = "imu";
  imu_marker.id = markers.markers.size() + 1;
  imu_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

  imu_marker.pose.orientation.w = 1.0;
  imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

  traj_marker.points.resize(keyframes.size());
  traj_marker.colors.resize(keyframes.size());
  for (int i = 0; i < keyframes.size(); i++) {
    Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
    traj_marker.points[i].x = pos.x();
    traj_marker.points[i].y = pos.y();
    traj_marker.points[i].z = pos.z();

    double p = static_cast<double>(i) / keyframes.size();
    traj_marker.colors[i].r = 1.0 - p;
    traj_marker.colors[i].g = p;
    traj_marker.colors[i].b = 0.0;
    traj_marker.colors[i].a = 1.0;

    if (keyframes[i]->acceleration) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
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
  traj_edge_marker.pose.orientation.w = 1.0;
  traj_edge_marker.scale.x = 0.05;

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
      color1.r = 1.0 - p1;
      color1.g = p1;
      color1.a = 1.0;

      color2.r = 1.0 - p2;
      color2.g = p2;
      color2.a = 1.0;
      traj_edge_marker.colors.push_back(color1);
      traj_edge_marker.colors.push_back(color2);

      // if(std::abs(v1->id() - v2->id()) > 2) {
      //   traj_edge_marker.points[i * 2].z += 0.5 + keyframe_h;
      //   traj_edge_marker.points[i * 2 + 1].z += 0.5 + keyframe_h;
      // }
    }
  }
  markers.markers.push_back(traj_edge_marker);

  // keyframe plane edge markers
  visualization_msgs::msg::Marker traj_plane_edge_marker;
  traj_plane_edge_marker.header.frame_id = keyframes_layer_id;
  traj_plane_edge_marker.header.stamp = stamp;
  traj_plane_edge_marker.ns = "keyframe_plane_edges";
  traj_plane_edge_marker.id = markers.markers.size();
  traj_plane_edge_marker.lifetime = duration_planes;
  traj_plane_edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
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
    }
  }
  markers.markers.push_back(traj_plane_edge_marker);

  // Wall edge markers
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
  markers.markers.push_back(sphere_marker);

  // x vertical plane markers
  visualization_msgs::msg::Marker x_vert_plane_marker;
  x_vert_plane_marker.pose.orientation.w = 1.0;
  x_vert_plane_marker.scale.x = 0.05;
  x_vert_plane_marker.scale.y = 0.05;
  x_vert_plane_marker.scale.z = 0.05;
  // plane_marker.points.resize(vert_planes.size());
  x_vert_plane_marker.header.frame_id = walls_layer_id;
  x_vert_plane_marker.header.stamp = stamp;
  x_vert_plane_marker.ns = "x_vert_planes";
  x_vert_plane_marker.id = markers.markers.size();
  x_vert_plane_marker.lifetime = duration_planes;
  x_vert_plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

  for (int i = 0; i < x_plane_snapshot.size(); ++i) {
    double p = static_cast<double>(i) / x_plane_snapshot.size();
    std_msgs::msg::ColorRGBA color;
    color.r = x_plane_snapshot[i].color[0] / 255;
    color.g = x_plane_snapshot[i].color[1] / 255;
    color.b = x_plane_snapshot[i].color[2] / 255;
    color.a = 0.5;
    for (size_t j = 0; j < x_plane_snapshot[i].cloud_seg_map->size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = x_plane_snapshot[i].cloud_seg_map->points[j].x;
      point.y = x_plane_snapshot[i].cloud_seg_map->points[j].y;
      point.z = x_plane_snapshot[i].cloud_seg_map->points[j].z;
      x_vert_plane_marker.points.push_back(point);
      x_vert_plane_marker.colors.push_back(color);
    }
  }
  markers.markers.push_back(x_vert_plane_marker);

  // y vertical plane markers
  visualization_msgs::msg::Marker y_vert_plane_marker;
  y_vert_plane_marker.pose.orientation.w = 1.0;
  y_vert_plane_marker.scale.x = 0.05;
  y_vert_plane_marker.scale.y = 0.05;
  y_vert_plane_marker.scale.z = 0.05;
  // plane_marker.points.resize(vert_planes.size());
  y_vert_plane_marker.header.frame_id = walls_layer_id;
  y_vert_plane_marker.header.stamp = stamp;
  y_vert_plane_marker.ns = "y_vert_planes";
  y_vert_plane_marker.id = markers.markers.size();
  y_vert_plane_marker.lifetime = duration_planes;
  y_vert_plane_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;

  for (int i = 0; i < y_plane_snapshot.size(); ++i) {
    double p = static_cast<double>(i) / y_plane_snapshot.size();
    std_msgs::msg::ColorRGBA color;
    color.r = y_plane_snapshot[i].color[0] / 255;
    color.g = y_plane_snapshot[i].color[1] / 255;
    color.b = y_plane_snapshot[i].color[2] / 255;
    color.a = 0.5;
    for (size_t j = 0; j < y_plane_snapshot[i].cloud_seg_map->size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = y_plane_snapshot[i].cloud_seg_map->points[j].x;
      point.y = y_plane_snapshot[i].cloud_seg_map->points[j].y;
      point.z = y_plane_snapshot[i].cloud_seg_map->points[j].z;
      y_vert_plane_marker.points.push_back(point);
      y_vert_plane_marker.colors.push_back(color);
    }
  }
  markers.markers.push_back(y_vert_plane_marker);

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
  hort_plane_marker.id = markers.markers.size();
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
  markers.markers.push_back(hort_plane_marker);

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
      x_infinite_room_line_marker.ns = "infinite_room_x_lines";
      x_infinite_room_line_marker.header.frame_id = rooms_layer_id;
      x_infinite_room_line_marker.header.stamp = stamp;
      x_infinite_room_line_marker.id = markers.markers.size() + 1;
      x_infinite_room_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      x_infinite_room_line_marker.color.r = color_r;
      x_infinite_room_line_marker.color.g = color_g;
      x_infinite_room_line_marker.color.b = color_b;
      x_infinite_room_line_marker.color.a = 1.0;
      x_infinite_room_line_marker.lifetime = duration_room;
    } else {
      x_infinite_room_snapshot[i].id = -1;
      x_infinite_room_line_marker.ns = "overlapped_infinite_room_x_lines";
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
    markers.markers.push_back(x_infinite_room_line_marker);

    // x infinite_room cube
    visualization_msgs::msg::Marker infinite_room_pose_marker;
    infinite_room_pose_marker.scale.x = 0.5;
    infinite_room_pose_marker.scale.y = 0.5;
    infinite_room_pose_marker.scale.z = 0.5;
    // plane_marker.points.resize(vert_planes.size());
    infinite_room_pose_marker.header.frame_id = rooms_layer_id;
    infinite_room_pose_marker.header.stamp = stamp;
    if (!overlapped_infinite_room) {
      infinite_room_pose_marker.ns = "x_infinite_room";
      infinite_room_pose_marker.id = markers.markers.size();
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
      markers.markers.push_back(infinite_room_pose_marker);
      /* room clusters */
      int cluster_id = 0;
      for (auto& cluster : x_infinite_room_snapshot[i].cluster_array.markers) {
        cluster.header.frame_id = walls_layer_id;
        if (cluster_id == 0) cluster.ns = "x_infinite_vertex";
        if (cluster_id == 1) cluster.ns = "x_infinite_vertex_edges";
        cluster.id = markers.markers.size() + 1;
        markers.markers.push_back(cluster);
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
      y_infinite_room_line_marker.ns = "infinite_room_y_lines";
      y_infinite_room_line_marker.header.frame_id = rooms_layer_id;
      y_infinite_room_line_marker.header.stamp = stamp;
      y_infinite_room_line_marker.id = markers.markers.size() + 1;
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
    markers.markers.push_back(y_infinite_room_line_marker);

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
      infinite_room_pose_marker.ns = "y_infinite_room";
      infinite_room_pose_marker.id = markers.markers.size();
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
      markers.markers.push_back(infinite_room_pose_marker);
      /* room clusters */
      int cluster_id = 0;
      for (auto& cluster : y_infinite_room_snapshot[i].cluster_array.markers) {
        cluster.header.frame_id = walls_layer_id;
        if (cluster_id == 0) cluster.ns = "y_infinite_vertex";
        if (cluster_id == 1) cluster.ns = "y_infinite_vertex_edges";
        cluster.id = markers.markers.size() + 1;
        markers.markers.push_back(cluster);
      }
    } else
      infinite_room_pose_marker.ns = "overlapped_y_infinite_room";
  }

  // room markers
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

    room_marker.pose.position.x = room_snapshot[i].node->estimate().translation()(0);
    room_marker.pose.position.y = room_snapshot[i].node->estimate().translation()(1);
    room_marker.pose.position.z = room_snapshot[i].node->estimate().translation()(2);
    Eigen::Quaterniond quat(room_snapshot[i].node->estimate().linear());
    room_marker.pose.orientation.x = quat.x();
    room_marker.pose.orientation.y = quat.y();
    room_marker.pose.orientation.z = quat.z();
    room_marker.pose.orientation.w = quat.w();
    room_marker.lifetime = duration_room;
    markers.markers.push_back(room_marker);

    // fill in the line marker
    visualization_msgs::msg::Marker room_line_marker;
    room_line_marker.scale.x = 0.02;
    room_line_marker.pose.orientation.w = 1.0;
    room_line_marker.ns = "rooms_line";
    room_line_marker.header.frame_id = rooms_layer_id;
    room_line_marker.header.stamp = stamp;
    room_line_marker.id = markers.markers.size() + 1;
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

    markers.markers.push_back(room_line_marker);

    /* room clusters */
    int cluster_id = 0;
    for (auto& cluster : room_snapshot[i].cluster_array.markers) {
      cluster.header.frame_id = walls_layer_id;
      if (cluster_id == 0) cluster.ns = "room_vertex";
      if (cluster_id == 1) cluster.ns = "room_edges";
      cluster.id = markers.markers.size() + 1;
      markers.markers.push_back(cluster);
    }
  }

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
      floor_marker.ns = "floors";
      floor_marker.id = markers.markers.size();
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
      floor_line_marker.ns = "floor_lines";
      floor_line_marker.header.frame_id = floors_layer_id;
      floor_line_marker.header.stamp = stamp;
      floor_line_marker.id = markers.markers.size() + 1;
      floor_line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      floor_line_marker.color.r = color_r;
      floor_line_marker.color.g = color_g;
      floor_line_marker.color.b = color_b;
      floor_line_marker.color.a = 1.0;
      floor_line_marker.lifetime = duration_floor;

      for (const auto& room : room_snapshot) {
        if (room.sub_room) continue;
        geometry_msgs::msg::Point p1, p2;
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
      markers.markers.push_back(floor_marker);
      markers.markers.push_back(floor_line_marker);
    }
  }

  return markers;
}

}  // namespace s_graphs
