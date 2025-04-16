#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphNode {
  using Ptr = std::shared_ptr<PoseGraphNode>;
  using ConstPtr = std::shared_ptr<const PoseGraphNode>;

  std::string frame_id;
  uint64_t stamp_ns = 0;
  int32_t robot_id = 0;
  uint64_t key = 0;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphNode& node);
};

}  // namespace pose_graph_tools
