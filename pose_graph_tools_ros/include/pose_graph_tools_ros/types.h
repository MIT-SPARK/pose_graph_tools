/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#pragma once

#include <queue>

#include <pose_graph_tools_msgs/PoseGraph.h>

namespace pose_graph_tools {

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
} // namespace pose_graph_tools