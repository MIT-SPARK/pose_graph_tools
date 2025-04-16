#pragma once

#include <memory>
#include <vector>

#include "pose_graph_tools/pose_graph_edge.h"
#include "pose_graph_tools/pose_graph_node.h"

namespace pose_graph_tools {

struct PoseGraph {
  using Ptr = std::shared_ptr<PoseGraph>;
  using ConstPtr = std::shared_ptr<const PoseGraph>;

  std::string frame_id;
  uint64_t stamp_ns = 0;
  std::vector<PoseGraphNode> nodes;
  std::vector<PoseGraphEdge> edges;

  inline bool empty() const { return nodes.empty() && edges.empty(); }
  friend std::ostream& operator<<(std::ostream& os, const PoseGraph& graph);
};

}  // namespace pose_graph_tools
