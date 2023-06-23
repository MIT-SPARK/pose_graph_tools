#pragma once

#include <memory>

#include "pose_graph_tools/bow_vector.h"

namespace pose_graph_tools {

struct BowQuery {
  using Ptr = std::shared_ptr<BowQuery>;
  using ConstPtr = std::shared_ptr<const BowQuery>;

  unsigned int robot_id;
  unsigned int pose_id;
  BowVector bow_vector;
};

}  // namespace pose_graph_tools
