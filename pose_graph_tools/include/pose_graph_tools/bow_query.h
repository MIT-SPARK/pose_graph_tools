#pragma once

#include "pose_graph_tools/bow_vector.h"

namespace pose_graph_tools {

struct BowQuery {
  unsigned int robot_id;
  unsigned int pose_id;
  BowVector bow_vector;
};

}  // namespace pose_graph_tools
