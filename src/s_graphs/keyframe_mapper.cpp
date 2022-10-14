// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/keyframe_mapper.hpp>

namespace s_graphs {

KeyframeMapper::KeyframeMapper(const ros::NodeHandle& private_nh) {
  nh = private_nh;
  max_keyframes_per_update = nh.param<int>("max_keyframes_per_update", 10);
  inf_calclator.reset(new InformationMatrixCalculator(nh));
}

KeyframeMapper::~KeyframeMapper() {}

int KeyframeMapper::map_keyframes(std::unique_ptr<GraphSLAM>& graph_slam, Eigen::Isometry3d odom2map, std::deque<KeyFrame::Ptr>& keyframe_queue, std::vector<KeyFrame::Ptr>& keyframes, std::deque<KeyFrame::Ptr>& new_keyframes, g2o::VertexSE3*& anchor_node, g2o::EdgeSE3*& anchor_edge, std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash>& keyframe_hash) {
  int num_processed = 0;
  for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
    num_processed = i;

    const auto& keyframe = keyframe_queue[i];
    // new_keyframes will be tested later for loop closure
    new_keyframes.push_back(keyframe);

    // add pose node
    Eigen::Isometry3d odom = odom2map * keyframe->odom;
    keyframe->node = graph_slam->add_se3_node(odom);
    keyframe_hash[keyframe->stamp] = keyframe;

    // fix the first node
    if(keyframes.empty() && new_keyframes.size() == 1) {
      if(nh.param<bool>("fix_first_node", false)) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        std::stringstream sst(nh.param<std::string>("fix_first_node_stddev", "1 1 1 1 1 1"));
        for(int j = 0; j < 6; j++) {
          double stddev = 1.0;
          sst >> stddev;
          inf(j, j) = 1.0 / stddev;
        }

        anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
        anchor_node->setFixed(true);
        anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
      }
    }

    if(i == 0 && keyframes.empty()) {
      continue;
    }

    // add edge between consecutive keyframes
    const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

    Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
    Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
    auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
    graph_slam->add_robust_kernel(edge, nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
  }
  return num_processed;
}

}  // namespace s_graphs