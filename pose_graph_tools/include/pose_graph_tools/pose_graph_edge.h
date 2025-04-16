#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphEdge {
  using Ptr = std::shared_ptr<PoseGraphEdge>;
  using ConstPtr = std::shared_ptr<const PoseGraphEdge>;
  using Covariance = Eigen::Matrix<double, 6, 6>;

  enum Type : int {
    UNKNOWN = -1,
    ODOM = 0,
    LOOPCLOSE = 1,
    LANDMARK = 2,
    REJECTED_LOOPCLOSE = 3,
    MESH = 4,
    POSE_MESH = 5,
    MESH_POSE = 6,
    PRIOR = 7,
    REJECTED_PRIOR = 8,
  } type = Type::UNKNOWN;

  uint64_t key_from = 0;
  uint64_t key_to = 0;

  int32_t robot_from = 0;
  int32_t robot_to = 0;

  // Optional timestamp the edge was computed at
  uint64_t stamp_ns = 0;

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  // Use ROS convention of negative covariance to indicate unset
  Covariance covariance = Covariance::Constant(-1.0);

  friend std::ostream& operator<<(std::ostream& os, const PoseGraphEdge& edge);
};

}  // namespace pose_graph_tools
