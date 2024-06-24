#pragma once

#include <memory>
#include <vector>

namespace pose_graph_tools {

struct BowVector {
  using Ptr = std::shared_ptr<BowVector>;
  using ConstPtr = std::shared_ptr<const BowVector>;

  std::vector<uint32_t> word_ids;
  std::vector<float> word_values;

  friend std::ostream& operator<<(std::ostream& os,
                                  const BowVector& bow_vector);
};

}  // namespace pose_graph_tools
