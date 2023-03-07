/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <pose_graph_tools/PoseGraph.h>

namespace pose_graph_tools {
bool savePoseGraphEdgesToFile(const PoseGraph &graph,
                              const std::string &filename);

// Filter duplicate edges in the input pose graph
// Two edges are considered duplicate if they share the common key_from, key_to,
// robot_from, robot_to
PoseGraph filterDuplicateEdges(const PoseGraph &graph_in);

} // namespace pose_graph_tools
