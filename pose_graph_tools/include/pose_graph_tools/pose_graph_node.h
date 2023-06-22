#pragma once

#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphNode {
  int robot_id;
  size_t key;
  Eigen::Affine3f pose;
};

}  // namespace pose_graph_tools
