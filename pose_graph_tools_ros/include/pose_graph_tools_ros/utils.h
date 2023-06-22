#pragma once

#include <string>

#include <pose_graph_tools_msgs/PoseGraph.h>

namespace pose_graph_tools {

bool savePoseGraphEdgesToFile(const pose_graph_tools_msgs::PoseGraph &graph,
                              const std::string &filename);

// Filter duplicate edges in the input pose graph
// Two edges are considered duplicate if they share the common key_from, key_to,
// robot_from, robot_to
pose_graph_tools_msgs::PoseGraph filterDuplicateEdges(
    const pose_graph_tools_msgs::PoseGraph &graph_in);

}  // namespace pose_graph_tools
