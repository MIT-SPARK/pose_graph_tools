#pragma once

#include <memory>
#include <vector>

#include "pose_graph_tools/pose_graph_edge.h"
#include "pose_graph_tools/pose_graph_node.h"

namespace pose_graph_tools {

struct PoseGraph {
  using Ptr = std::shared_ptr<PoseGraph>;
  using ConstPtr = std::shared_ptr<const PoseGraph>;

  std::vector<PoseGraphNode> nodes;
  std::vector<PoseGraphEdge> edges;
};

}  // namespace pose_graph_tools
