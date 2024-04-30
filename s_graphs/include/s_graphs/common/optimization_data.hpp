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

#ifndef OPTIMIZATION_DATA_H
#define OPTIMIZATION_DATA_H

#include <iosfwd>
#include <sstream>
#include <string>

#include "g2o/core/optimizable_graph.h"

namespace s_graphs {
class OptimizationData : public g2o::HyperGraph::Data {
 public:
  OptimizationData() {}
  ~OptimizationData() {}

  bool read(std::istream& is) {
    std::cout << "read not implemented" << std::endl;
    return false;
  }

  bool write(std::ostream& os) const {
    std::cout << "write not implemented" << std::endl;
    return false;
  }

  void set_edge_info(bool value) { artificial_edge = value; }

  void get_edge_info(bool& value) { value = artificial_edge; }

  void get_rep_node_info(bool& value) { value = rep_node; }

  void set_rep_node_info(bool value) {
    rep_node = value;
    if (rep_node) marginalized = false;
  }

  void get_stair_node_info(bool& value) { value = stair_node; }

  void set_stair_node_info(bool value) { stair_node = value; }

  void get_loop_closure_info(bool& value) { value = loop_closure; }

  void set_loop_closure_info(bool value) {
    loop_closure = value;
    if (loop_closure) marginalized = false;
  }

  void get_marginalized_info(bool& value) { value = marginalized; }

  void set_marginalized_info(bool value) {
    if (loop_closure) {
      marginalized = false;
    } else {
      marginalized = value;
    }
  }

  void set_anchor_node_info(bool value) { anchor_node = value; }

  void get_anchor_node_info(bool& value) { value = anchor_node; }

 private:
  bool marginalized = false;
  bool loop_closure = false;
  bool anchor_node = false;
  bool rep_node = false;
  bool artificial_edge = false;
  bool stair_node = false;
};
}  // namespace s_graphs
#endif  // OPTIMIZATION_DATA_HPP
