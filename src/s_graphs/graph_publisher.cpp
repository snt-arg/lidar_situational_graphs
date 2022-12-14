#include <s_graphs/graph_publisher.hpp>

GraphPublisher::GraphPublisher(const ros::NodeHandle& private_nh) {}

GraphPublisher::~GraphPublisher() {}
ros1_graph_manager_interface::Graph GraphPublisher::publish_graph(std::unique_ptr<s_graphs::GraphSLAM>& graph_slam, std::string graph_type) {
  g2o::SparseOptimizer* local_graph = graph_slam->graph.get();
  std::vector<ros1_graph_manager_interface::Edge> edges_vec;
  std::vector<ros1_graph_manager_interface::Node> nodes_vec;
  ros1_graph_manager_interface::Graph graph_msg;
  std::vector<ros1_graph_manager_interface::Attribute> edge_att_vec;
  std::vector<ros1_graph_manager_interface::Attribute> node_att_vec;

  // Graph Type
  if(graph_type == "BIM") {
    graph_msg.name = "BIM";
  } else {
    graph_msg.name = "ONLINE";
  }

  // iterate over graph
  auto edge_itr = local_graph->edges().begin();
  for(int i = 0; edge_itr != local_graph->edges().end(); edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *edge_itr;
    g2o::EdgeRoom4Planes* edge_r4p = dynamic_cast<g2o::EdgeRoom4Planes*>(edge);
    g2o::EdgeRoom2Planes* edge_r2p = dynamic_cast<g2o::EdgeRoom2Planes*>(edge);
    // g2o::EdgePlaneParallel* edge_pp = dynamic_cast<g2o::EdgePlaneParallel*>(edge);
    if(edge_r4p) {
      g2o::VertexRoomXYLB* v_room = dynamic_cast<g2o::VertexRoomXYLB*>(edge_r4p->vertices()[0]);
      g2o::VertexPlane* v_xplane1 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[1]);
      g2o::VertexPlane* v_xplane2 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[2]);
      g2o::VertexPlane* v_yplane1 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[3]);
      g2o::VertexPlane* v_yplane2 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[4]);
      ros1_graph_manager_interface::Edge graph_edge;
      ros1_graph_manager_interface::Node graph_node;
      ros1_graph_manager_interface::Attribute edge_attribute;
      ros1_graph_manager_interface::Attribute node_attribute;

      // Room Node
      graph_node.id = v_room->id();
      graph_node.type = "Finite Room";
      node_attribute.name = "VertexRoomXYLB";
      Eigen::Vector2d room_pose = v_room->estimate();
      node_attribute.fl_value.push_back(room_pose.x());
      node_attribute.fl_value.push_back(room_pose.y());
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 1 node
      graph_node.id = v_xplane1->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      Eigen::Vector4d plane_coeffs = v_xplane1->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 2 node
      graph_node.id = v_xplane2->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      plane_coeffs = v_xplane2->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 3 node
      graph_node.id = v_yplane1->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      plane_coeffs = v_yplane1->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 4 node
      graph_node.id = v_yplane2->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      plane_coeffs = v_yplane2->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // first edge
      graph_edge.origin_node = v_room->id();
      graph_edge.target_node = v_xplane1->id();
      edge_attribute.name = "EdgeRoom4Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 2nd edge
      graph_edge.target_node = v_xplane2->id();
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 3rd edge
      graph_edge.target_node = v_yplane1->id();
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // 4th edge
      graph_edge.target_node = v_yplane2->id();
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
      // std::cout << "v_room : " << v_room->id() << std::endl;
      // std::cout << "v_xplane1 : " << v_xplane1->id() << '\n';
      // std::cout << "v_xplane2 : " << v_xplane2->id() << '\n';
      // std::cout << "v_yplane1 : " << v_yplane1->id() << '\n';
      // std::cout << "v_yplane2 : " << v_yplane2->id() << '\n';
    } else if(edge_r2p) {
      g2o::VertexRoomXYLB* v_room = dynamic_cast<g2o::VertexRoomXYLB*>(edge_r2p->vertices()[0]);
      g2o::VertexPlane* v_xplane = dynamic_cast<g2o::VertexPlane*>(edge_r2p->vertices()[1]);
      g2o::VertexPlane* v_yplane = dynamic_cast<g2o::VertexPlane*>(edge_r2p->vertices()[2]);
      ros1_graph_manager_interface::Node graph_node;
      ros1_graph_manager_interface::Attribute edge_attribute;
      ros1_graph_manager_interface::Attribute node_attribute;

      // Room Node
      graph_node.id = v_room->id();
      graph_node.type = "Infinte Room";
      node_attribute.name = "VertexRoomXYLB";
      Eigen::Vector2d room_pose = v_room->estimate();
      node_attribute.fl_value.push_back(room_pose.x());
      node_attribute.fl_value.push_back(room_pose.y());
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 1 node
      graph_node.id = v_xplane->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      Eigen::Vector4d plane_coeffs = v_xplane->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Plane 2 node
      graph_node.id = v_yplane->id();
      graph_node.type = "Plane";
      node_attribute.name = "VertexPlane";
      plane_coeffs = v_yplane->estimate().coeffs();
      node_attribute.fl_value.push_back(plane_coeffs(0));
      node_attribute.fl_value.push_back(plane_coeffs(1));
      node_attribute.fl_value.push_back(plane_coeffs(2));
      node_attribute.fl_value.push_back(plane_coeffs(3));
      node_att_vec.push_back(node_attribute);
      graph_node.attributes = node_att_vec;
      nodes_vec.push_back(graph_node);
      node_attribute.fl_value.clear();
      node_att_vec.clear();

      // Edge 1
      ros1_graph_manager_interface::Edge graph_edge;
      graph_edge.origin_node = v_room->id();
      graph_edge.target_node = v_xplane->id();
      edge_attribute.name = "EdgeRoom2Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();

      // Edge 2
      graph_edge.origin_node = v_room->id();
      graph_edge.target_node = v_yplane->id();
      edge_attribute.name = "EdgeRoom2Planes";
      edge_att_vec.push_back(edge_attribute);
      graph_edge.attributes = edge_att_vec;
      edges_vec.push_back(graph_edge);
      edge_att_vec.clear();
      // } else if(edge_pp) {
      //   g2o::VertexPlane* v_plane1 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[0]);
      //   g2o::VertexPlane* v_plane2 = dynamic_cast<g2o::VertexPlane*>(edge_r4p->vertices()[1]);
      // }
    }
    // std::cout << " edges_vec size : " << edges_vec.size() << std::endl;
  }
  graph_msg.edges = edges_vec;
  graph_msg.nodes = nodes_vec;
  edges_vec.clear();
  nodes_vec.clear();
  return graph_msg;
}