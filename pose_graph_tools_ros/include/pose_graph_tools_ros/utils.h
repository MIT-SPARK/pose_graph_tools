#pragma once

#include <queue>
#include <string>
#include <vector>

#include <pose_graph_tools_msgs/PoseGraph.h>

namespace pose_graph_tools {

bool savePoseGraphEdgesToFile(const pose_graph_tools_msgs::PoseGraph &graph,
                              const std::string &filename);

// Filter duplicate edges in the input pose graph
// Two edges are considered duplicate if they share the common key_from, key_to,
// robot_from, robot_to
pose_graph_tools_msgs::PoseGraph filterDuplicateEdges(
    const pose_graph_tools_msgs::PoseGraph &graph_in);

// Buffers
class PoseGraphStampCompare {
 public:
  bool operator()(pose_graph_tools_msgs::PoseGraphConstPtr x,
                  pose_graph_tools_msgs::PoseGraphConstPtr y) {
    return x->header.stamp > y->header.stamp;
  }
};

typedef std::priority_queue<pose_graph_tools_msgs::PoseGraphConstPtr,
                            std::vector<pose_graph_tools_msgs::PoseGraphConstPtr>,
                            PoseGraphStampCompare>
    StampedQueue;

}  // namespace pose_graph_tools
