#pragma once

#include <vector>

#include "pose_graph_tools/pose_graph_edge.h"
#include "pose_graph_tools/pose_graph_node.h"

namespace pose_graph_tools {

struct PoseGraph {
  std::vector<PoseGraphNode> nodes;
  std::vector<PoseGraphEdge> edges;
};

}  // namespace pose_graph_tools
