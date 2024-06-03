#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include <s_graphs/common/information_matrix_calculator.hpp>

namespace s_graphs {

InformationMatrixCalculator::InformationMatrixCalculator(
    const rclcpp::Node::SharedPtr node) {
  use_const_inf_matrix =
      node->get_parameter("use_const_inf_matrix").get_parameter_value().get<bool>();
  const_stddev_x =
      node->get_parameter("const_stddev_x").get_parameter_value().get<double>();
  const_stddev_q =
      node->get_parameter("const_stddev_q").get_parameter_value().get<double>();

  var_gain_a = node->get_parameter("var_gain_a").get_parameter_value().get<double>();
  min_stddev_x =
      node->get_parameter("min_stddev_x").get_parameter_value().get<double>();
  max_stddev_x =
      node->get_parameter("max_stddev_x").get_parameter_value().get<double>();
  min_stddev_q =
      node->get_parameter("min_stddev_q").get_parameter_value().get<double>();
  max_stddev_q =
      node->get_parameter("max_stddev_q").get_parameter_value().get<double>();
  fitness_score_thresh =
      node->get_parameter("fitness_score_thresh").get_parameter_value().get<double>();
}

InformationMatrixCalculator::~InformationMatrixCalculator() {}

Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(
    const pcl::PointCloud<PointT>::ConstPtr& cloud1,
    const pcl::PointCloud<PointT>::ConstPtr& cloud2,
    const Eigen::Isometry3d& relpose) const {
  if (use_const_inf_matrix || cloud1->points.empty() || cloud2->points.empty()) {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= const_stddev_x;
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
    return inf;
  }

  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  float w_x =
      weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q =
      weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}

double InformationMatrixCalculator::calc_fitness_score(
    const pcl::PointCloud<PointT>::ConstPtr& cloud1,
    const pcl::PointCloud<PointT>::ConstPtr& cloud2,
    const Eigen::Isometry3d& relpose,
    double max_range) {
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;
  pcl::transformPointCloud(*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size(); ++i) {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range) {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max());
}

}  // namespace s_graphs
