#pragma once

#include <vector>

#include "pose_graph_tools/bow_query.h"

namespace pose_graph_tools {

struct BowQueries {
  unsigned int destination_robot_id;
  std::vector<BowQuery> queries;
};

}  // namespace pose_graph_tools
