// SPDX-License-Identifier: BSD-2-Clause

#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <pcl/registration/registration.h>

namespace s_graphs {

struct registration_params {
 public:
  std::string registration_method;
  long int reg_num_threads;
  double reg_transformation_epsilon;
  long int reg_maximum_iterations;
  double reg_max_correspondence_distance;
  long int reg_correspondence_randomness;
  double reg_resolution;
  bool reg_use_reciprocal_correspondences;
  long int reg_max_optimizer_iterations;
  std::string reg_nn_search_method;
};

/**
 * @brief Select a scan matching algorithm according to rosparams.
 *
 * @param registration_params
 * @return selected scan matching
 */
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>>
select_registration_method(registration_params params);

}  // namespace s_graphs

#endif  //
