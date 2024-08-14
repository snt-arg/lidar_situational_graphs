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

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <s_graphs/visualization/graph_publisher.hpp>
#include <vector>

#include "g2o/vertex_room.hpp"

GraphPublisher::GraphPublisher() {}

GraphPublisher::~GraphPublisher() {}
situational_graphs_reasoning_msgs::msg::Graph GraphPublisher::publish_graph(
    const g2o::SparseOptimizer* local_graph,
    std::string graph_type,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes_prior,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes_prior,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec_prior,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::unordered_map<int, s_graphs::Rooms>& rooms_vec,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::unordered_map<int, s_graphs::InfiniteRooms>& y_infinite_rooms) {
  std::vector<situational_graphs_reasoning_msgs::msg::Edge> edges_vec;
  std::vector<situational_graphs_reasoning_msgs::msg::Node> nodes_vec;
  situational_graphs_reasoning_msgs::msg::Graph graph_msg;
  std::vector<situational_graphs_reasoning_msgs::msg::Attribute> edge_att_vec;
  std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec;

  // Graph Type
  if (graph_type == "Prior") {
    graph_msg.name = "Prior";
    for (const auto& pair : x_vert_planes_prior) {
      const auto& plane_info = pair.second;

      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = plane_info.id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";

      Eigen::Vector4d plane_coeffs = plane_info.plane_node->estimate().coeffs();
      node_attribute.fl_value = {
          plane_coeffs(0), plane_coeffs(1), plane_coeffs(2), plane_coeffs(3)};

      std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec = {
          node_attribute};
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();
    }

    for (const auto& pair : y_vert_planes_prior) {
      const auto& plane_info = pair.second;

      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = plane_info.id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";

      Eigen::Vector4d plane_coeffs = plane_info.plane_node->estimate().coeffs();
      node_attribute.fl_value = {
          plane_coeffs(0), plane_coeffs(1), plane_coeffs(2), plane_coeffs(3)};

      std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec = {
          node_attribute};
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();
    }

    for (const auto& pair : rooms_vec_prior) {
      const auto& room_info = pair.second;
      g2o::VertexRoom* v_room = room_info.node;
      situational_graphs_reasoning_msgs::msg::Edge graph_edge;
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
      graph_node.id = room_info.id;
      graph_node.type = "Finite Room";
      node_attribute.name = "Geometric_info";
      Eigen::Vector2d room_pose = v_room->estimate().translation().head(2);
      node_attribute.fl_value.push_back(room_pose.x());
      node_attribute.fl_value.push_back(room_pose.y());
      node_attribute.fl_value.push_back(0.0);
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // first edge
      graph_edge.origin_node = v_room->id();
      graph_edge.target_node = room_info.plane_x1_id;
      edge_attribute.name = "Geometric_info";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge

      graph_edge.target_node = room_info.plane_x2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge

      graph_edge.target_node = room_info.plane_y1_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = room_info.plane_y2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }

  } else {
    graph_msg.name = "Online";
    std::unordered_map<int, s_graphs::Rooms> publishable_rooms_vec;
    publishable_rooms_vec = rooms_vec;
    for (const auto& pair : x_vert_planes) {
      const auto& plane_info = pair.second;

      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = plane_info.id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";

      Eigen::Vector4d plane_coeffs = plane_info.plane_node->estimate().coeffs();
      node_attribute.fl_value = {
          plane_coeffs(0), plane_coeffs(1), plane_coeffs(2), plane_coeffs(3)};

      std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec = {
          node_attribute};
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();
    }
    for (const auto& pair : y_vert_planes) {
      const auto& plane_info = pair.second;

      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = plane_info.id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";

      Eigen::Vector4d plane_coeffs = plane_info.plane_node->estimate().coeffs();
      node_attribute.fl_value = {
          plane_coeffs(0), plane_coeffs(1), plane_coeffs(2), plane_coeffs(3)};

      std::vector<situational_graphs_reasoning_msgs::msg::Attribute> node_att_vec = {
          node_attribute};
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();
    }
    std::vector<int> room_keys;
    for (const auto& pair : publishable_rooms_vec) {
      room_keys.push_back(pair.first);
    }

    for (int j = 0; j < room_keys.size(); ++j) {
      publishable_rooms_vec[room_keys[j]].sub_room = false;
    }

    for (int i = 0; i < room_keys.size(); i++) {
      if (publishable_rooms_vec[room_keys[i]].sub_room) continue;

      for (auto& pair : publishable_rooms_vec) {
        auto& room = pair.second;
        if (room.id == publishable_rooms_vec[room_keys[i]].id) continue;

        float dist_room_room = std::sqrt(
            std::pow(
                room.node->estimate().translation()(0) -
                    publishable_rooms_vec[room_keys[i]].node->estimate().translation()(
                        0),
                2) +
            std::pow(
                room.node->estimate().translation()(1) -
                    publishable_rooms_vec[room_keys[i]].node->estimate().translation()(
                        1),
                2));

        if (dist_room_room < 2.0 && room.sub_room == false) {
          room.sub_room = true;
        }
      }

      g2o::VertexRoom* v_room = publishable_rooms_vec[room_keys[i]].node;
      situational_graphs_reasoning_msgs::msg::Edge graph_edge;
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

      graph_node.id = publishable_rooms_vec[room_keys[i]].id;
      graph_node.type = "Finite Room";
      node_attribute.name = "Geometric_info";
      Eigen::Vector2d room_pose = v_room->estimate().translation().head(2);
      node_attribute.fl_value.push_back(room_pose.x());
      node_attribute.fl_value.push_back(room_pose.y());
      node_attribute.fl_value.push_back(0.0);
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // first edge
      graph_edge.origin_node = v_room->id();
      graph_edge.target_node = publishable_rooms_vec[room_keys[i]].plane_x1_id;
      edge_attribute.name = "EdgeRoom4Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge
      graph_edge.target_node = publishable_rooms_vec[room_keys[i]].plane_x2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge
      graph_edge.target_node = publishable_rooms_vec[room_keys[i]].plane_y1_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = publishable_rooms_vec[room_keys[i]].plane_y2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }

    auto edge_itr = local_graph->edges().begin();
    for (int i = 0; edge_itr != local_graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeRoom4Planes* edge_r4p = dynamic_cast<g2o::EdgeRoom4Planes*>(edge);
      g2o::EdgeRoom2Planes* edge_r2p = dynamic_cast<g2o::EdgeRoom2Planes*>(edge);
      g2o::Edge2Planes* edge_2p = dynamic_cast<g2o::Edge2Planes*>(edge);
      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);

      if (edge_2p) {
        situational_graphs_reasoning_msgs::msg::Node graph_node;
        situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
        g2o::VertexPlane* v_plane1 =
            dynamic_cast<g2o::VertexPlane*>(edge_2p->vertices()[0]);
        g2o::VertexPlane* v_plane2 =
            dynamic_cast<g2o::VertexPlane*>(edge_2p->vertices()[1]);
        // Plane 1 node
        auto found_vertex1 =
            std::find_if(nodes_vec.begin(),
                         nodes_vec.end(),
                         boost::bind(&situational_graphs_reasoning_msgs::msg::Node::id,
                                     _1) == v_plane1->id());
        if (found_vertex1 == nodes_vec.end()) {
          graph_node.id = v_plane1->id();
          graph_node.type = "Plane";
          node_attribute.name = "Geometric_info";
          Eigen::Vector4d plane_coeffs = v_plane1->estimate().coeffs();
          node_attribute.fl_value.push_back(plane_coeffs(0));
          node_attribute.fl_value.push_back(plane_coeffs(1));
          node_attribute.fl_value.push_back(plane_coeffs(2));
          node_attribute.fl_value.push_back(plane_coeffs(3));
          node_att_vec.push_back(node_attribute);
          graph_node.attributes = node_att_vec;
          nodes_vec.push_back(graph_node);
          node_attribute.fl_value.clear();
          node_att_vec.clear();
        }
        auto found_vertex2 =
            std::find_if(nodes_vec.begin(),
                         nodes_vec.end(),
                         boost::bind(&situational_graphs_reasoning_msgs::msg::Node::id,
                                     _1) == v_plane2->id());
        if (found_vertex2 == nodes_vec.end()) {
          graph_node.id = v_plane2->id();
          graph_node.type = "Plane";
          node_attribute.name = "Geometric_info";
          Eigen::Vector4d plane_coeffs = v_plane2->estimate().coeffs();
          node_attribute.fl_value.push_back(plane_coeffs(0));
          node_attribute.fl_value.push_back(plane_coeffs(1));
          node_attribute.fl_value.push_back(plane_coeffs(2));
          node_attribute.fl_value.push_back(plane_coeffs(3));
          node_att_vec.push_back(node_attribute);
          graph_node.attributes = node_att_vec;
          nodes_vec.push_back(graph_node);
          node_attribute.fl_value.clear();
          node_att_vec.clear();
        }
      }
    }
  }
  graph_msg.edges = edges_vec;
  graph_msg.nodes = nodes_vec;
  edges_vec.clear();
  nodes_vec.clear();
  return graph_msg;
}

situational_graphs_reasoning_msgs::msg::GraphKeyframes
GraphPublisher::publish_graph_keyframes(
    const g2o::SparseOptimizer* local_graph,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  situational_graphs_reasoning_msgs::msg::GraphKeyframes msg;
  msg.keyframes.reserve(keyframes.size());
  for (const auto& keyframe_pair : keyframes) {
    msg.keyframes.emplace_back(s_graphs::Keyframe2ROS(*(keyframe_pair.second)));
  }
  return msg;
}

// std::vector<g2o::HyperGraph::Edge*> get_planes_from_room(
//     g2o::VertexRoom* _room_ptr,
//     const std::vector<s_graphs::Rooms>& rooms_vec) {
//   std::vector<g2o::HyperGraph::Edge*> out_vec;
//   auto room_edges_iter = _room_ptr->edges().begin();
//   for (int i = 0; room_edges_iter != _room_ptr->edges().end(); room_edges_iter++,
//   i++) {
//     g2o::HyperGraph::Edge* edge = *room_edges_iter;
//     g2o::EdgeSE3Plane* keyframe_to_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
//   }
// }
//
//

// std::find_if(x_vert_planes.begin(), x_vert_planes.end, funct());
// std::vector<>

// auto room_edges_iter = node->edges().begin();
// for (int i = 0; room_edges_iter != node->edges().end(); room_edges_iter++, i++)
// {
//   g2o::HyperGraph::Edge* edge = *room_edges_iter;
//   g2o::EdgeSE3Plane* keyframe_to_plane =
//   dynamic_cast<g2o::EdgeSE3Plane*>(edge);
// }
/* auto traj_edge_itr = local_graph->edges().begin();
for (int i = 0; traj_edge_itr != local_graph->edges().end(); traj_edge_itr++, i++)
{ g2o::HyperGraph::Edge* edge = *traj_edge_itr; g2o::VertexRoom* room =

      g2o::EdgeSE3Plane* keyframe_to_plane =
dynamic_cast<g2o::EdgeSE3Plane*>(edge);

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

    double p1 = static_cast<double>(v1->id()) / local_graph->vertices().size();
    double p2 = static_cast<double>(v2->id()) / local_graph->vertices().size();

    std_msgs::msg::ColorRGBA color1, color2;
    color1.r = 1.0 - p1;
    color1.g = p1;
    color1.a = 1.0;

    color2.r = 1.0 - p2;
    color2.g = p2;
    color2.a = 1.0;

    // if(std::abs(v1->id() - v2->id()) > 2) {
    //   traj_edge_marker.points[i * 2].z += 0.5 + keyframe_h;
    //   traj_edge_marker.points[i * 2 + 1].z += 0.5 + keyframe_h;
    // }
  }
} */
// }
