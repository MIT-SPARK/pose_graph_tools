#pragma once

#include "pose_graph_tools/bow_vector.h"

namespace pose_graph_tools {

struct BoWQuery {
  unsigned int robot_id;
  unsigned int pose_id;
  BoWVector bow_vector;
};

}  // namespace pose_graph_tools
