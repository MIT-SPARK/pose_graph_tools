#pragma once

#include <memory>

#include "pose_graph_tools/bow_vector.h"

namespace pose_graph_tools {

struct BowQuery {
  using Ptr = std::shared_ptr<BowQuery>;
  using ConstPtr = std::shared_ptr<const BowQuery>;

  uint32_t robot_id;
  uint32_t pose_id;
  BowVector bow_vector;

  friend std::ostream& operator<<(std::ostream& os, const BowQuery& query);
};

}  // namespace pose_graph_tools
