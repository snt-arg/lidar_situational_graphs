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

#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <s_graphs/common/point_types.hpp>

#include "rclcpp/rclcpp.hpp"

namespace s_graphs {

/**
 * @brief
 */
class InformationMatrixCalculator {
 public:
  InformationMatrixCalculator() {}
  InformationMatrixCalculator(const rclcpp::Node::SharedPtr node);
  ~InformationMatrixCalculator();

  template <typename ParamServer>

  /**
   * @brief
   *
   * @param params
   */
  void load(ParamServer& params) {
    use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
    const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
    const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

    var_gain_a = params.template param<double>("var_gain_a", 20.0);
    min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
    max_stddev_x = params.template param<double>("max_stddev_x", 5.0);
    min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
    max_stddev_q = params.template param<double>("max_stddev_q", 0.2);
    fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
  }

  /**
   * @brief
   *
   * @param cloud1
   * @param cloud2
   * @param relpose
   * @param max_range
   * @return
   */
  static double calc_fitness_score(
      const pcl::PointCloud<PointT>::ConstPtr& cloud1,
      const pcl::PointCloud<PointT>::ConstPtr& cloud2,
      const Eigen::Isometry3d& relpose,
      double max_range = std::numeric_limits<double>::max());

  /**
   * @brief
   *
   * @param cloud1
   * @param cloud2
   * @param relpose
   * @return Information matrix
   */
  Eigen::MatrixXd calc_information_matrix(
      const pcl::PointCloud<PointT>::ConstPtr& cloud1,
      const pcl::PointCloud<PointT>::ConstPtr& cloud2,
      const Eigen::Isometry3d& relpose) const;

 private:
  /**
   * @brief
   *
   * @param a
   * @param max_x
   * @param min_y
   * @param max_y
   * @param x
   * @return
   */
  double weight(double a, double max_x, double min_y, double max_y, double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

 private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
};

}  // namespace s_graphs

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
