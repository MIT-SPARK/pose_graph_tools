#pragma once

#include <memory>
#include <vector>

namespace pose_graph_tools {

struct BowVector {
  using Ptr = std::shared_ptr<BowVector>;
  using ConstPtr = std::shared_ptr<const BowVector>;

  std::vector<unsigned int> word_ids;
  std::vector<float> word_values;
};

}  // namespace pose_graph_tools
