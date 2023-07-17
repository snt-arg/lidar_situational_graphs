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

  void get_marginalized_info(bool& value) { value = marginalized; }

  void set_marginalize_info(bool value) { marginalized = value; }

 private:
  bool marginalized = false;
};
}  // namespace s_graphs
#endif  // OPTIMIZATION_DATA_HPP
