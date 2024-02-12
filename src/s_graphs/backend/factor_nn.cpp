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
FactorNN::FactorNN(std::string _factor_type) {
  if (_factor_type == "room"){
    path = "/home/adminpc/reasoning_ws/src/graph_factor_nn/torchscripts/room.pt";
  } else if (_factor_type == "wall") {
    path = "/home/adminpc/reasoning_ws/src/graph_factor_nn/torchscripts/wall.pt";
  }
  
  try {
    module = torch::jit::load(path);
  }
  catch (const c10::Error& e) {
    std::cerr << "Error loading the model: " << e.what() << std::endl;
    // return -1;
  }
}

Eigen::Vector2d FactorNN::infer(std::vector<std::vector<float>> input_vector){
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  std::vector<float> zeros(input_vector[0].size(), 0.0f);
  input_vector.push_back(zeros);
  int rows = input_vector.size();
  int cols = input_vector.empty() ? 0 : input_vector[0].size();
  std::vector<float> flatVec;
  for (const auto& subVec : input_vector) {
      flatVec.insert(flatVec.end(), subVec.begin(), subVec.end());
  }

  std::vector<int64_t> edgeIndexVector;
  for (int i = 0; i <= rows - 2; i += 1) {
      edgeIndexVector.push_back(i);
    }
  for (int i = 0; i <= rows - 2; i += 1) {
      edgeIndexVector.push_back(rows - 1);
    }
  std::vector<int64_t> batchVector(rows, 0.0f);

  torch::Tensor xTensor = torch::from_blob(flatVec.data(), {rows, cols}, torch::kFloat);
  // x.push_back(xTensor);
  torch::jit::IValue ivalue_x = xTensor;

  torch::Tensor edgeIndexTensor = torch::from_blob(edgeIndexVector.data(), {2, rows - 1}, torch::kInt64);
  torch::jit::IValue ivalue_edgeIndex = edgeIndexTensor;

  torch::Tensor batchTensor = torch::from_blob(batchVector.data(), {rows}, torch::kInt64);
  torch::jit::IValue ivalue_batch = batchTensor;

  std::vector<torch::jit::IValue> inputs = {ivalue_x, ivalue_edgeIndex, ivalue_batch};   

  at::Tensor output = module.forward(inputs ).toTensor();
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "NN Time difference = " << std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() << "[µs]" << std::endl;

  Eigen::Vector2d eigenVector;
  for (int i = 0; i < 2; ++i) {
    eigenVector[i] = output[0][i].item<double>();
  }
  return eigenVector;
}
}
