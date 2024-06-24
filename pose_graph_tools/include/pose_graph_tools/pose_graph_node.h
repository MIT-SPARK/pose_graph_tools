#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphNode {
  using Ptr = std::shared_ptr<PoseGraphNode>;
  using ConstPtr = std::shared_ptr<const PoseGraphNode>;

  uint64_t stamp_ns;
  int32_t robot_id;
  uint64_t key;
  Eigen::Affine3d pose;

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphNode& node);
};

}  // namespace pose_graph_tools
