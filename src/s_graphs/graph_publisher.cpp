#include <s_graphs/graph_publisher.hpp>

GraphPublisher::GraphPublisher(const ros::NodeHandle& private_nh) {}

GraphPublisher::~GraphPublisher() {}
void GraphPublisher::publish_graph(std::unique_ptr<s_graphs::GraphSLAM>& graph_slam) {
  g2o::SparseOptimizer* local_graph = graph_slam->graph.get();
  auto edge_itr = local_graph->edges().begin();
  for(int i = 0; edge_itr != local_graph->edges().end(); edge_itr++, i++) {
    g2o::HyperGraph::Edge* edge = *edge_itr;
    g2o::EdgeRoom4Planes* edge_r4p = dynamic_cast<g2o::EdgeRoom4Planes*>(edge);
    if(edge_r4p) {
      g2o::VertexRoomXYLB* v_room;
      g2o::VertexPlane* v_xplane1;
      g2o::VertexPlane* v_xplane2;
      g2o::VertexPlane* v_yplane1;
      g2o::VertexPlane* v_yplane2;
      std::cout << "room edge found !  :  " << i << '\n';
    }
  }
  //   auto vertex_itr = local_graph->vertices().begin();
  //   for(int i = 0; vertex_itr != local_graph->edges().end(); vertex_itr++, i++) {
  //     g2o::HyperGraph::Edge* edge = *vertex_itr;
  //     g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
  //     if(edge_se3) {
  //       g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
  //       g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
  //       Eigen::Vector3d pt1 = v1->estimate().translation();
  //       Eigen::Vector3d pt2 = v2->estimate().translation();
  //     }
  //   }

  //   g2o::VertexSE3*
}