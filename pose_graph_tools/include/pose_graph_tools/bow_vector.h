#pragma once

#include <vector>

namespace pose_graph_tools {

struct BowVector {
  std::vector<unsigned int> word_ids;
  std::vector<float> word_values;
};

}  // namespace pose_graph_tools
