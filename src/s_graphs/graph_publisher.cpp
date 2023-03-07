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

#include <s_graphs/graph_publisher.hpp>

#include "s_graphs/ros_utils.hpp"

GraphPublisher::GraphPublisher() {}

GraphPublisher::~GraphPublisher() {}
graph_manager_msgs::msg::Graph GraphPublisher::publish_graph(
    const g2o::SparseOptimizer* local_graph,
    std::string graph_type,
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes_prior,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes_prior,
    const std::vector<s_graphs::Rooms>& rooms_vec_prior,
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes,
    const std::vector<s_graphs::Rooms>& rooms_vec,
    const std::vector<s_graphs::InfiniteRooms>& x_infinite_rooms,
    const std::vector<s_graphs::InfiniteRooms>& y_infinite_rooms) {
  std::vector<graph_manager_msgs::msg::Edge> edges_vec;
  std::vector<graph_manager_msgs::msg::Node> nodes_vec;
  graph_manager_msgs::msg::Graph graph_msg;
  std::vector<graph_manager_msgs::msg::Attribute> edge_att_vec;
  std::vector<graph_manager_msgs::msg::Attribute> node_att_vec;

  // Graph Type
  if (graph_type == "Prior") {
    graph_msg.name = "Prior";
    for (int i = 0; i < x_vert_planes_prior.size(); i++) {
      g2o::Plane3D v_plane = x_vert_planes_prior[i].plane;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = x_vert_planes_prior[i].id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";
      Eigen::Vector4d plane_coeffs = v_plane.coeffs();
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
    for (int i = 0; i < y_vert_planes_prior.size(); i++) {
      g2o::Plane3D v_plane = y_vert_planes_prior[i].plane;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = y_vert_planes_prior[i].id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";
      Eigen::Vector4d plane_coeffs = v_plane.coeffs();
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
    for (int i = 0; i < rooms_vec_prior.size(); i++) {
      g2o::VertexRoomXYLB* v_room = rooms_vec_prior[i].node;
      graph_manager_msgs::msg::Edge graph_edge;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute edge_attribute;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = rooms_vec_prior[i].id;
      graph_node.type = "Finite Room";
      node_attribute.name = "Geometric_info";
      Eigen::Vector2d room_pose = v_room->estimate();
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
      graph_edge.target_node = rooms_vec_prior[i].plane_x1_id;
      edge_attribute.name = "Geometric_info";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge

      graph_edge.target_node = rooms_vec_prior[i].plane_x2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge

      graph_edge.target_node = rooms_vec_prior[i].plane_y1_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = rooms_vec_prior[i].plane_y2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }

  } else {
    graph_msg.name = "ONLINE";
    for (int i = 0; i < x_vert_planes.size(); i++) {
      g2o::Plane3D v_plane = x_vert_planes[i].plane;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = x_vert_planes[i].id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";
      Eigen::Vector4d plane_coeffs = v_plane.coeffs();
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
    for (int i = 0; i < y_vert_planes.size(); i++) {
      g2o::Plane3D v_plane = y_vert_planes[i].plane;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = y_vert_planes[i].id;
      graph_node.type = "Plane";
      node_attribute.name = "Geometric_info";
      Eigen::Vector4d plane_coeffs = v_plane.coeffs();
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
    for (int i = 0; i < rooms_vec.size(); i++) {
      g2o::VertexRoomXYLB* v_room = rooms_vec[i].node;
      graph_manager_msgs::msg::Edge graph_edge;
      graph_manager_msgs::msg::Node graph_node;
      graph_manager_msgs::msg::Attribute edge_attribute;
      graph_manager_msgs::msg::Attribute node_attribute;
      graph_node.id = rooms_vec[i].id;
      graph_node.type = "Finite Room";
      node_attribute.name = "Geometric_info";
      Eigen::Vector2d room_pose = v_room->estimate();
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
      graph_edge.target_node = rooms_vec[i].plane_x1_id;
      edge_attribute.name = "EdgeRoom4Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge

      graph_edge.target_node = rooms_vec[i].plane_x2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge

      graph_edge.target_node = rooms_vec[i].plane_y1_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = rooms_vec[i].plane_y2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }

    // for(int i = 0; i < x_infinite_rooms.size(); i++) {
    //   g2o::VertexRoomXYLB* v_room = x_infinite_rooms[i].node;
    //   graph_manager_msgs::msg::Edge graph_edge;
    //   graph_manager_msgs::msg::Node graph_node;
    //   graph_manager_msgs::msg::Attribute edge_attribute;
    //   graph_manager_msgs::msg::Attribute node_attribute;

    //   // X Infinte Room Node
    //   graph_node.id = x_infinite_rooms[i].id;
    //   graph_node.type = "Infinite Room";
    //   node_attribute.name = "X-Infinite Room";
    //   Eigen::Vector2d room_pose = v_room->estimate();
    //   node_attribute.fl_value.push_back(room_pose.x());
    //   node_attribute.fl_value.push_back(room_pose.y());
    //   node_attribute.fl_value.push_back(0.0);
    //   node_att_vec.push_back(node_attribute);
    //   graph_node.attributes = node_att_vec;
    //   nodes_vec.push_back(graph_node);
    //   node_attribute.fl_value.clear();
    //   node_att_vec.clear();

    //   // first edge
    //   graph_edge.origin_node = v_room->id();
    //   graph_edge.target_node = x_infinite_rooms[i].plane1_id;
    //   edge_attribute.name = "EdgeRoom2Planes";
    //   edge_att_vec.push_back(edge_attribute);
    //   graph_edge.attributes = edge_att_vec;
    //   edges_vec.push_back(graph_edge);
    //   edge_att_vec.clear();

    //   // 2nd edge

    //   graph_edge.target_node = x_infinite_rooms[i].plane2_id;
    //   edge_att_vec.push_back(edge_attribute);
    //   graph_edge.attributes = edge_att_vec;
    //   edges_vec.push_back(graph_edge);
    //   edge_att_vec.clear();
    // }

    // for(int i = 0; i < y_infinite_rooms.size(); i++) {
    //   g2o::VertexRoomXYLB* v_room = y_infinite_rooms[i].node;
    //   graph_manager_msgs::msg::Edge graph_edge;
    //   graph_manager_msgs::msg::Node graph_node;
    //   graph_manager_msgs::msg::Attribute edge_attribute;
    //   graph_manager_msgs::msg::Attribute node_attribute;

    //   // Y Infinte Room Node
    //   graph_node.id = y_infinite_rooms[i].id;
    //   graph_node.type = "Infinite Room";
    //   node_attribute.name = "Y-Infinite Room";
    //   Eigen::Vector2d room_pose = v_room->estimate();
    //   node_attribute.fl_value.push_back(room_pose.x());
    //   node_attribute.fl_value.push_back(room_pose.y());
    //   node_attribute.fl_value.push_back(0.0);
    //   node_att_vec.push_back(node_attribute);
    //   graph_node.attributes = node_att_vec;
    //   nodes_vec.push_back(graph_node);
    //   node_attribute.fl_value.clear();
    //   node_att_vec.clear();

    //   // first edge
    //   graph_edge.origin_node = v_room->id();
    //   graph_edge.target_node = y_infinite_rooms[i].plane1_id;
    //   edge_attribute.name = "EdgeRoom2Planes";
    //   edge_att_vec.push_back(edge_attribute);
    //   graph_edge.attributes = edge_att_vec;
    //   edges_vec.push_back(graph_edge);
    //   edge_att_vec.clear();

    //   // 2nd edge

    //   graph_edge.target_node = y_infinite_rooms[i].plane2_id;
    //   edge_att_vec.push_back(edge_attribute);
    //   graph_edge.attributes = edge_att_vec;
    //   edges_vec.push_back(graph_edge);
    //   edge_att_vec.clear();
    // }

    auto edge_itr = local_graph->edges().begin();
    for (int i = 0; edge_itr != local_graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeRoom4Planes* edge_r4p = dynamic_cast<g2o::EdgeRoom4Planes*>(edge);
      g2o::EdgeRoom2Planes* edge_r2p = dynamic_cast<g2o::EdgeRoom2Planes*>(edge);
      g2o::Edge2Planes* edge_2p = dynamic_cast<g2o::Edge2Planes*>(edge);
      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);

      if (edge_2p) {
        graph_manager_msgs::msg::Node graph_node;
        graph_manager_msgs::msg::Attribute node_attribute;
        g2o::VertexPlane* v_plane1 =
            dynamic_cast<g2o::VertexPlane*>(edge_2p->vertices()[0]);
        g2o::VertexPlane* v_plane2 =
            dynamic_cast<g2o::VertexPlane*>(edge_2p->vertices()[1]);
        // Plane 1 node
        auto found_vertex1 = std::find_if(
            nodes_vec.begin(),
            nodes_vec.end(),
            boost::bind(&graph_manager_msgs::msg::Node::id, _1) == v_plane1->id());
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
        auto found_vertex2 = std::find_if(
            nodes_vec.begin(),
            nodes_vec.end(),
            boost::bind(&graph_manager_msgs::msg::Node::id, _1) == v_plane2->id());
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

graph_manager_msgs::msg::GraphKeyframes GraphPublisher::publish_graph_keyframes(
    const g2o::SparseOptimizer* local_graph,
    const std::vector<s_graphs::KeyFrame::Ptr>& keyframes) {
  graph_manager_msgs::msg::GraphKeyframes msg;
  msg.keyframes.reserve(keyframes.size());
  for (auto& keyframe : keyframes) {
    msg.keyframes.emplace_back(s_graphs::Keyframe2ROS(*keyframe));
  }
  return msg;
};

