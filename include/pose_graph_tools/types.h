/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <queue>

#include "pose_graph_tools/PoseGraph.h"

namespace pose_graph_tools {

// Buffers
class PoseGraphStampCompare {
public:
  bool operator()(pose_graph_tools::PoseGraphConstPtr x,
                  pose_graph_tools::PoseGraphConstPtr y) {
    return x->header.stamp > y->header.stamp;
  }
};

typedef std::priority_queue<pose_graph_tools::PoseGraphConstPtr,
                            std::vector<pose_graph_tools::PoseGraphConstPtr>,
                            PoseGraphStampCompare>
    StampedQueue;
} // namespace pose_graph_tools