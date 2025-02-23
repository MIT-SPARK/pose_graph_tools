#pragma once

#include <queue>
#include <string>
#include <vector>

#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/time.hpp>

namespace pose_graph_tools {

using PoseGraphMsg = pose_graph_tools_msgs::msg::PoseGraph;
using PoseGraphMsgPtr = pose_graph_tools_msgs::msg::PoseGraph::ConstSharedPtr;

bool savePoseGraphEdgesToFile(const PoseGraphMsg& graph,
                              const std::string& filename);

// Filter duplicate edges in the input pose graph
// Two edges are considered duplicate if they share the common key_from, key_to,
// robot_from, robot_to
PoseGraphMsg filterDuplicateEdges(const PoseGraphMsg& graph_in);

// Buffers
class PoseGraphStampCompare {
 public:
  bool operator()(PoseGraphMsgPtr x, PoseGraphMsgPtr y) {
    return rclcpp::Time(x->header.stamp) > rclcpp::Time(y->header.stamp);
  }
};

using StampedQueue = std::priority_queue<PoseGraphMsgPtr,
                                         std::vector<PoseGraphMsgPtr>,
                                         PoseGraphStampCompare>;

}  // namespace pose_graph_tools
