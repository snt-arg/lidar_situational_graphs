#include <s_graphs/backend/loop_mapper.hpp>

namespace s_graphs {

LoopMapper::LoopMapper(const rclcpp::Node::SharedPtr node) {
  inf_calclator.reset(new InformationMatrixCalculator(node));
}

LoopMapper::~LoopMapper() {}

void LoopMapper::add_loops(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                           const std::vector<Loop::Ptr>& loops,
                           std::mutex& graph_mutex) {
  for (const auto& loop : loops) {
    Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
    Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(
        loop->key1->cloud, loop->key2->cloud, relpose);
    graph_mutex.lock();
    std::cout << "loop found between keyframes " << loop->key1->node->id() << " and "
              << loop->key2->node->id() << std::endl;

    set_data(loop->key1->node);
    set_data(loop->key2->node);

    g2o::EdgeLoopClosure* edge = covisibility_graph->add_loop_closure_edge(
        loop->key1->node, loop->key2->node, relpose, information_matrix);
    covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    graph_mutex.unlock();
  }
}

void LoopMapper::set_data(g2o::VertexSE3* keyframe_node) {
  auto current_key_data = dynamic_cast<OptimizationData*>(keyframe_node->userData());
  if (current_key_data) {
    current_key_data->set_loop_closure_info(true);
  } else {
    OptimizationData* data = new OptimizationData();
    data->set_loop_closure_info(true);
    keyframe_node->setUserData(data);
  }
}

}  // namespace s_graphs