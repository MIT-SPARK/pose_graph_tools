/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#pragma once

#include <pose_graph_tools/PoseGraph.h>

namespace pose_graph_tools {
bool savePoseGraphMsgToFile(const PoseGraph& graph,
                            const std::string& filename);

}  // namespace pose_graph_tools
