#ifndef LOOP_MAPPER_HPP
#define LOOP_MAPPER_HPP

#include <g2o/edge_loop_closure.hpp>
#include <s_graphs/common/information_matrix_calculator.hpp>
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/frontend/loop_detector.hpp>

namespace s_graphs {

class LoopMapper {
 public:
  LoopMapper(const rclcpp::Node::SharedPtr node);
  ~LoopMapper();

 public:
  void add_loops(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                 const std::vector<Loop::Ptr>& loops,
                 std::mutex& graph_mutex);

 private:
  void set_data(g2o::VertexSE3* keyframe_node);

 private:
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};
}  // namespace s_graphs

#endif #LOOP_MAPPER_HPP