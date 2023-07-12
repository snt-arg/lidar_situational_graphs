#ifndef OPTIMIZATION_DATA_H
#define OPTIMIZATION_DATA_H

#include <iosfwd>
#include <string>

#include "g2o/core/optimizable_graph.h"

namespace s_graphs {
class OptimizationData : public g2o::HyperGraph::Data {
 public:
  OptimizationData() {}
  ~OptimizationData() {}

  bool read(std::istream& is) override {
    is >> marginalized;
    return true;
  }

  bool write(std::ostream& os) const override {
    os << marginalized;
    return os.good();
  }

 private:
  bool marginalized;
};
}  // namespace s_graphs
#endif  // OPTIMIZATION_DATA_HPP
