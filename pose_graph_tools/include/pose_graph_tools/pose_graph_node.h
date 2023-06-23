#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphNode {
  using Ptr = std::shared_ptr<PoseGraphNode>;
  using ConstPtr = std::shared_ptr<const PoseGraphNode>;

  int robot_id;
  size_t key;
  Eigen::Affine3f pose;
};

}  // namespace pose_graph_tools
