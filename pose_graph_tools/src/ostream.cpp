#include "pose_graph_tools/all.h"

namespace pose_graph_tools {

// TODO(lschmid): If important consider better pretty printing with linebreaks,
// naming the enums etc.

std::ostream& operator<<(std::ostream& os, const BowVector& bow_vector) {
  os << "BowVector{word_ids=[";
  for (const auto& word_id : bow_vector.word_ids) {
    os << word_id << ", ";
  }
  os << "], word_values=[";
  for (const auto& word_value : bow_vector.word_values) {
    os << word_value << ", ";
  }
  os << "]}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const BowQuery& query) {
  os << "BowQuery{robot_id=" << query.robot_id << ", pose_id=" << query.pose_id
     << ", bow_vector=" << query.bow_vector << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const BowQueries& queries) {
  os << "BowQueries{destination_robot_id=" << queries.destination_robot_id
     << ", queries=[";
  for (const auto& query : queries.queries) {
    os << query << ", ";
  }
  os << "]}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PoseGraphNode& node) {
  os << "PoseGraphNode{stamp_ns=" << node.stamp_ns
     << ", robot_id=" << node.robot_id << ", key=" << node.key
     << ", translation=[" << node.pose.translation().transpose()
     << "], rotation=["
     << Eigen::Quaterniond(node.pose.rotation()).coeffs().transpose() << "]}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PoseGraphEdge& edge) {
  os << "PoseGraphEdge{type=" << edge.type << ", key_from=" << edge.key_from
     << ", key_to=" << edge.key_to << ", robot_from=" << edge.robot_from
     << ", robot_to=" << edge.robot_to << ", stamp_ns=" << edge.stamp_ns
     << ", translation=[" << edge.pose.translation().transpose()
     << "], rotation=["
     << Eigen::Quaterniond(edge.pose.rotation()).coeffs().transpose() << "]}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PoseGraph& graph) {
  os << "PoseGraph{stamp_ns=" << graph.stamp_ns << ", nodes=[";
  for (const auto& node : graph.nodes) {
    os << node << ", ";
  }
  os << "], edges=[";
  for (const auto& edge : graph.edges) {
    os << edge << ", ";
  }
  os << "]}";
  return os;
}

}  // namespace pose_graph_tools
