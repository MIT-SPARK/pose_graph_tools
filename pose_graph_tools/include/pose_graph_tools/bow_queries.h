#pragma once

#include <memory>
#include <vector>

#include "pose_graph_tools/bow_query.h"

namespace pose_graph_tools {

struct BowQueries {
  using Ptr = std::shared_ptr<BowQueries>;
  using ConstPtr = std::shared_ptr<const BowQueries>;

  unsigned int destination_robot_id;
  std::vector<BowQuery> queries;
};

}  // namespace pose_graph_tools
