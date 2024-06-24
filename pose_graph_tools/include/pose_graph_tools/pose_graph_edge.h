#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphEdge {
  using Ptr = std::shared_ptr<PoseGraphEdge>;
  using ConstPtr = std::shared_ptr<const PoseGraphEdge>;

  enum Type : int {
    ODOM = 0,
    LOOPCLOSE = 1,
    LANDMARK = 2,
    REJECTED_LOOPCLOSE = 3,
    MESH = 4,
    POSE_MESH = 5,
    MESH_POSE = 6,
  };

  uint64_t key_from;
  uint64_t key_to;

  int32_t robot_from;
  int32_t robot_to;

  Type type;
  uint64_t stamp_ns;

  Eigen::Affine3d pose;
  Eigen::Matrix<double, 6, 6> covariance;

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphEdge& edge);
};

}  // namespace pose_graph_tools
