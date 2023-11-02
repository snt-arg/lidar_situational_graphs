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

// SPDX-License-Identifier: BSD-2-Clause

#include <torch/script.h>
#include <s_graphs/backend/factor_nn.hpp>

#include <iostream>
#include <memory>
#include <chrono>

namespace s_graphs {

// FactorNN::FactorNN(const rclcpp::Node::SharedPtr node) {
FactorNN::FactorNN() {
  path = "/home/adminpc/reasoning_ws/src/graph_reasoning/torchscripts/test.pt";
  try {
    module = torch::jit::load(path);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    // return -1;
  }
}

Eigen::Vector2d FactorNN::infer(std::vector<float> input_vector){
  // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  torch::Tensor torchTensor = torch::from_blob(input_vector.data(), {1, input_vector.size()}, torch::kFloat);
  std::vector<torch::IValue> ivalueVector;
  ivalueVector.push_back(torchTensor);
  at::Tensor output = module.forward(ivalueVector).toTensor();
  // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "NN Time difference = " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[µs]" << std::endl;

  Eigen::Vector2d eigenVector;
  for (int i = 0; i < 2; ++i) {
    eigenVector[i] = output[0][i].item<double>();
  }
  return eigenVector;
}
}