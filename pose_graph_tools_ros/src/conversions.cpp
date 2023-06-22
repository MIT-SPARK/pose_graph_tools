#include "pose_graph_tools_ros/conversions.h"

namespace pose_graph_tools {

geometry_msgs::Pose toMsg(const Eigen::Affine3f& pose) {
  geometry_msgs::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  const Eigen::Quaternionf quaternion(pose.rotation());
  result.orientation.x = quaternion.x();
  result.orientation.y = quaternion.y();
  result.orientation.z = quaternion.z();
  result.orientation.w = quaternion.w();
  return result;
}

Eigen::Affine3f fromMsg(const geometry_msgs::Pose& pose) {
  Eigen::Affine3f result;
  result.translation().x() = pose.position.x;
  result.translation().y() = pose.position.y;
  result.translation().z() = pose.position.z;
  const Eigen::Quaternionf quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  result.linear() = quaternion.toRotationMatrix();
  return result;
}

pose_graph_tools_msgs::BowVector toMsg(const BowVector& bow_vector) {
  pose_graph_tools_msgs::BowVector result;
  result.word_ids = bow_vector.word_ids;
  result.word_values = bow_vector.word_values;
  return result;
}

BowVector fromMsg(const pose_graph_tools_msgs::BowVector& bow_vector) {
  BowVector result;
  result.word_ids = bow_vector.word_ids;
  result.word_values = bow_vector.word_values;
  return result;
}

pose_graph_tools_msgs::BowQuery toMsg(const BowQuery& bow_query) {
  pose_graph_tools_msgs::BowQuery result;
  result.robot_id = bow_query.robot_id;
  result.pose_id = bow_query.pose_id;
  result.bow_vector = toMsg(bow_query.bow_vector);
  return result;
}

BowQuery fromMsg(const pose_graph_tools_msgs::BowQuery& bow_query) {
  BowQuery result;
  result.robot_id = bow_query.robot_id;
  result.pose_id = bow_query.pose_id;
  result.bow_vector = fromMsg(bow_query.bow_vector);
  return result;
}

pose_graph_tools_msgs::BowQueries toMsg(const BowQueries& bow_queries) {
  pose_graph_tools_msgs::BowQueries result;
  result.destination_robot_id = bow_queries.destination_robot_id;
  result.queries.reserve(bow_queries.queries.size());
  for (const auto& query : bow_queries.queries) {
    result.queries.emplace_back(toMsg(query));
  }
  return result;
}

BowQueries fromMsg(const pose_graph_tools_msgs::BowQueries& bow_queries) {
  BowQueries result;
  result.destination_robot_id = bow_queries.destination_robot_id;
  result.queries.reserve(bow_queries.queries.size());
  for (const auto& query : bow_queries.queries) {
    result.queries.emplace_back(fromMsg(query));
  }
  return result;
}

pose_graph_tools_msgs::PoseGraphEdge toMsg(const PoseGraphEdge& pose_graph_edge) {
  pose_graph_tools_msgs::PoseGraphEdge result;
  result.key_from = pose_graph_edge.key_from;
  result.key_to = pose_graph_edge.key_to;
  result.robot_from = pose_graph_edge.robot_from;
  result.robot_to = pose_graph_edge.robot_to;
  result.type = static_cast<int>(pose_graph_edge.type);
  result.pose = toMsg(pose_graph_edge.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      result.covariance[r * 6 + c] = pose_graph_edge.covariance(r, c);
    }
  }
  return result;
}

PoseGraphEdge fromMsg(const pose_graph_tools_msgs::PoseGraphEdge& pose_graph_edge) {
  PoseGraphEdge result;
  result.key_from = pose_graph_edge.key_from;
  result.key_to = pose_graph_edge.key_to;
  result.robot_from = pose_graph_edge.robot_from;
  result.robot_to = pose_graph_edge.robot_to;
  result.type = static_cast<PoseGraphEdge::Type>(pose_graph_edge.type);
  result.pose = fromMsg(pose_graph_edge.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      result.covariance(r, c) = pose_graph_edge.covariance[r * 6 + c];
    }
  }
  return result;
}

pose_graph_tools_msgs::PoseGraphNode toMsg(const PoseGraphNode& pose_graph_node) {
  pose_graph_tools_msgs::PoseGraphNode result;
  result.key = pose_graph_node.key;
  result.robot_id = pose_graph_node.robot_id;
  result.pose = toMsg(pose_graph_node.pose);
  return result;
}

PoseGraphNode fromMsg(const pose_graph_tools_msgs::PoseGraphNode& pose_graph_node) {
  PoseGraphNode result;
  result.key = pose_graph_node.key;
  result.robot_id = pose_graph_node.robot_id;
  result.pose = fromMsg(pose_graph_node.pose);
  return result;
}

pose_graph_tools_msgs::PoseGraph toMsg(const PoseGraph& pose_graph) {
  pose_graph_tools_msgs::PoseGraph result;
  result.nodes.reserve(pose_graph.nodes.size());
  for (const auto& node : pose_graph.nodes) {
    result.nodes.emplace_back(toMsg(node));
  }
  result.edges.reserve(pose_graph.edges.size());
  for (const auto& edge : pose_graph.edges) {
    result.edges.emplace_back(toMsg(edge));
  }
  return result;
}

PoseGraph fromMsg(const pose_graph_tools_msgs::PoseGraph& pose_graph) {
  PoseGraph result;
  result.nodes.reserve(pose_graph.nodes.size());
  for (const auto& node : pose_graph.nodes) {
    result.nodes.emplace_back(fromMsg(node));
  }
  result.edges.reserve(pose_graph.edges.size());
  for (const auto& edge : pose_graph.edges) {
    result.edges.emplace_back(fromMsg(edge));
  }
  return result;
}

}  // namespace pose_graph_tools
