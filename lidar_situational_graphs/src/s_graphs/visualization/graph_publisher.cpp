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
    const std::vector<s_graphs::VerticalPlanes>& x_vert_planes_prior,
    const std::vector<s_graphs::VerticalPlanes>& y_vert_planes_prior,
    const std::vector<s_graphs::Rooms>& rooms_vec_prior,
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
    for (int i = 0; i < x_vert_planes_prior.size(); i++) {
      g2o::Plane3D v_plane = x_vert_planes_prior[i].plane_node->estimate();
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
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
      g2o::Plane3D v_plane = y_vert_planes_prior[i].plane_node->estimate();
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
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
      g2o::VertexRoom* v_room = rooms_vec_prior[i].node;
      situational_graphs_reasoning_msgs::msg::Edge graph_edge;
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
      graph_node.id = rooms_vec_prior[i].id;
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
    for (const auto& x_vert_plane : x_vert_planes) {
      g2o::Plane3D v_plane = x_vert_plane.second.plane_node->estimate();
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
      graph_node.id = x_vert_plane.second.id;
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
    for (const auto& y_vert_plane : y_vert_planes) {
      g2o::Plane3D v_plane = y_vert_plane.second.plane_node->estimate();
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
      graph_node.id = y_vert_plane.second.id;
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
    for (const auto& room : rooms_vec) {
      g2o::VertexRoom* v_room = room.second.node;
      situational_graphs_reasoning_msgs::msg::Edge graph_edge;
      situational_graphs_reasoning_msgs::msg::Node graph_node;
      situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
      situational_graphs_reasoning_msgs::msg::Attribute node_attribute;
      graph_node.id = room.second.id;
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
      graph_edge.target_node = room.second.plane_x1_id;
      edge_attribute.name = "EdgeRoom4Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge

      graph_edge.target_node = room.second.plane_x2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge

      graph_edge.target_node = room.second.plane_y1_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = room.second.plane_y2_id;
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
    }

    // for(int i = 0; i < x_infinite_rooms.size(); i++) {
    //   g2o::VertexRoom* v_room = x_infinite_rooms[i].node;
    //   situational_graphs_reasoning_msgs::msg::Edge graph_edge;
    //   situational_graphs_reasoning_msgs::msg::Node graph_node;
    //   situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
    //   situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

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
    //   g2o::VertexRoom* v_room = y_infinite_rooms[i].node;
    //   situational_graphs_reasoning_msgs::msg::Edge graph_edge;
    //   situational_graphs_reasoning_msgs::msg::Node graph_node;
    //   situational_graphs_reasoning_msgs::msg::Attribute edge_attribute;
    //   situational_graphs_reasoning_msgs::msg::Attribute node_attribute;

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
  for (auto& keyframe : keyframes) {
    // bool kf_marginalized =
    //    s_graphs::GraphUtils::get_keyframe_marg_data(keyframe.second->node);
    // if (!kf_marginalized)
    msg.keyframes.emplace_back(s_graphs::Keyframe2ROS(*keyframe.second));
  }
  return msg;
}

situational_graphs_reasoning_msgs::msg::GraphKeyframes
GraphPublisher::publish_graph_keyframes(
    const g2o::SparseOptimizer* local_graph,
    const std::vector<s_graphs::KeyFrame::Ptr>& keyframes) {
  situational_graphs_reasoning_msgs::msg::GraphKeyframes msg;
  msg.keyframes.reserve(keyframes.size());
  for (auto& keyframe : keyframes) {
    // bool kf_marginalized =
    // s_graphs::GraphUtils::get_keyframe_marg_data(keyframe->node); if
    // (!kf_marginalized)
    msg.keyframes.emplace_back(s_graphs::Keyframe2ROS(*keyframe));
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
