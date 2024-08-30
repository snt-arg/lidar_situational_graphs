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

#ifndef REGISTRATIONS_HPP
#define REGISTRATIONS_HPP

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <iostream>
#include <s_graphs/common/point_types.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif
#include <s_graphs/common/point_types.hpp>

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
pcl::Registration<PointT, PointT>::Ptr select_registration_method(
    registration_params params);

}  // namespace s_graphs

#endif  // REGISTRATIONS_HPP
