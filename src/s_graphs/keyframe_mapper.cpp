// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/keyframe_mapper.hpp>

namespace s_graphs {

KeyframeMapper::KeyframeMapper(const rclcpp::Node::SharedPtr node) {
  node_obj = node;
  max_keyframes_per_update = node_obj->get_parameter("max_keyframes_per_update")
                                 .get_parameter_value()
                                 .get<int>();

  inf_calclator.reset(new InformationMatrixCalculator(node_obj));
}

KeyframeMapper::~KeyframeMapper() {}

int KeyframeMapper::map_keyframes(
    std::shared_ptr<GraphSLAM>& graph_slam,
    Eigen::Isometry3d odom2map,
    std::deque<KeyFrame::Ptr>& keyframe_queue,
    std::vector<KeyFrame::Ptr>& keyframes,
    std::deque<KeyFrame::Ptr>& new_keyframes,
    g2o::VertexSE3*& anchor_node,
    g2o::EdgeSE3*& anchor_edge,
    std::unordered_map<rclcpp::Time, KeyFrame::Ptr, RosTimeHash>& keyframe_hash) {
  int num_processed = 0;
  for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update);
       i++) {
    num_processed = i;

    const auto& keyframe = keyframe_queue[i];
    // new_keyframes will be tested later for loop closure
    new_keyframes.push_back(keyframe);

    // add pose node
    Eigen::Isometry3d odom = odom2map * keyframe->odom;
    keyframe->node = graph_slam->add_se3_node(odom);
    keyframe_hash[keyframe->stamp] = keyframe;

    // fix the first node
    if (keyframes.empty() && new_keyframes.size() == 1) {
      node_obj->declare_parameter("fix_first_node", false);
      if (node_obj->get_parameter("fix_first_node").get_parameter_value().get<bool>()) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        node_obj->declare_parameter("fix_first_node_stddev", "1 1 1 1 1 1");
        std::stringstream sst(node_obj->get_parameter("fix_first_node_stddev")
                                  .get_parameter_value()
                                  .get<std::string>());
        for (int j = 0; j < 6; j++) {
          double stddev = 1.0;
          sst >> stddev;
          inf(j, j) = 1.0 / stddev;
        }

        anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
        anchor_node->setFixed(true);
        anchor_edge = graph_slam->add_se3_edge(
            anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
      }
    }

    if (i == 0 && keyframes.empty()) {
      continue;
    }

    // add edge between consecutive keyframes
    const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

    Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
        keyframe->cloud, prev_keyframe->cloud, relative_pose);
    auto edge = graph_slam->add_se3_edge(
        keyframe->node, prev_keyframe->node, relative_pose, information);
    graph_slam->add_robust_kernel(edge, "Huber", 1.0);
  }
  return num_processed;
}

}  // namespace s_graphs