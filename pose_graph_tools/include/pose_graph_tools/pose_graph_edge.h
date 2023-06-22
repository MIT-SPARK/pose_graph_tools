#pragma once

#include <vector>

#include <Eigen/Dense>

namespace pose_graph_tools {

struct PoseGraphEdge {
  enum Type : int {
    ODOM = 0,
    LOOPCLOSE = 1,
    LANDMARK = 2,
    REJECTED_LOOPCLOSE = 3,
    MESH = 4,
    POSE_MESH = 5,
    MESH_POSE = 6,
  };

  size_t key_from;
  size_t key_to;

  int robot_from;
  int robot_to;

  Type type;

  Eigen::Matrix<double, 6, 6> covariance;
};

}  // namespace pose_graph_tools
