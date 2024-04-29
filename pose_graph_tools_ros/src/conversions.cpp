#include "pose_graph_tools_ros/conversions.h"

#include <tf2_eigen/tf2_eigen.h>

namespace pose_graph_tools {

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

pose_graph_tools_msgs::PoseGraphEdge toMsg(
    const PoseGraphEdge& pose_graph_edge) {
  pose_graph_tools_msgs::PoseGraphEdge result;
  result.key_from = pose_graph_edge.key_from;
  result.key_to = pose_graph_edge.key_to;
  result.robot_from = pose_graph_edge.robot_from;
  result.robot_to = pose_graph_edge.robot_to;
  result.type = static_cast<int>(pose_graph_edge.type);
  result.header.stamp.fromNSec(pose_graph_edge.stamp_ns);
  tf2::convert(pose_graph_edge.pose, result.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      result.covariance[r * 6 + c] = pose_graph_edge.covariance(r, c);
    }
  }
  return result;
}

PoseGraphEdge fromMsg(
    const pose_graph_tools_msgs::PoseGraphEdge& pose_graph_edge) {
  PoseGraphEdge result;
  result.key_from = pose_graph_edge.key_from;
  result.key_to = pose_graph_edge.key_to;
  result.robot_from = pose_graph_edge.robot_from;
  result.robot_to = pose_graph_edge.robot_to;
  result.stamp_ns = pose_graph_edge.header.stamp.toNSec();
  result.type = static_cast<PoseGraphEdge::Type>(pose_graph_edge.type);
  tf2::convert(pose_graph_edge.pose, result.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      result.covariance(r, c) = pose_graph_edge.covariance[r * 6 + c];
    }
  }
  return result;
}

pose_graph_tools_msgs::PoseGraphNode toMsg(
    const PoseGraphNode& pose_graph_node) {
  pose_graph_tools_msgs::PoseGraphNode result;
  result.header.stamp.fromNSec(pose_graph_node.stamp_ns);
  result.key = pose_graph_node.key;
  result.robot_id = pose_graph_node.robot_id;
  tf2::convert(pose_graph_node.pose, result.pose);
  return result;
}

PoseGraphNode fromMsg(
    const pose_graph_tools_msgs::PoseGraphNode& pose_graph_node) {
  PoseGraphNode result;
  result.stamp_ns = pose_graph_node.header.stamp.toNSec();
  result.key = pose_graph_node.key;
  result.robot_id = pose_graph_node.robot_id;
  tf2::convert(pose_graph_node.pose, result.pose);
  return result;
}

pose_graph_tools_msgs::PoseGraph toMsg(const PoseGraph& pose_graph) {
  pose_graph_tools_msgs::PoseGraph result;
  result.header.stamp.fromNSec(pose_graph.stamp_ns);
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
  result.stamp_ns = pose_graph.header.stamp.toNSec();
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
