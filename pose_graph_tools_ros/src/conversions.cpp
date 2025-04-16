#include "pose_graph_tools_ros/conversions.h"

#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace pose_graph_tools {

void toMsg(const BowVector& src, pose_graph_msgs::BowVector& dest) {
  dest.word_ids = src.word_ids;
  dest.word_values = src.word_values;
}

void fromMsg(const pose_graph_msgs::BowVector& src, BowVector& dest) {
  dest.word_ids = src.word_ids;
  dest.word_values = src.word_values;
}

void toMsg(const BowQuery& src, pose_graph_msgs::BowQuery& dest) {
  dest.robot_id = src.robot_id;
  dest.pose_id = src.pose_id;
  toMsg(src.bow_vector, dest.bow_vector);
}

void fromMsg(const pose_graph_msgs::BowQuery& src, BowQuery& dest) {
  dest.robot_id = src.robot_id;
  dest.pose_id = src.pose_id;
  fromMsg(src.bow_vector, dest.bow_vector);
}

void toMsg(const BowQueries& src, pose_graph_msgs::BowQueries& dest) {
  dest.destination_robot_id = src.destination_robot_id;
  dest.queries.reserve(src.queries.size());
  for (const auto& query : src.queries) {
    auto& dest_query = dest.queries.emplace_back();
    toMsg(query, dest_query);
  }
}

void fromMsg(const pose_graph_msgs::BowQueries& src, BowQueries& dest) {
  dest.destination_robot_id = src.destination_robot_id;
  dest.queries.reserve(src.queries.size());
  for (const auto& query : src.queries) {
    auto& dest_query = dest.queries.emplace_back();
    fromMsg(query, dest_query);
  }
}

void toMsg(const PoseGraphEdge& src, pose_graph_msgs::PoseGraphEdge& dest) {
  dest.type = static_cast<int>(src.type);
  dest.key_from = src.key_from;
  dest.key_to = src.key_to;
  dest.robot_from = src.robot_from;
  dest.robot_to = src.robot_to;
  dest.header.stamp = rclcpp::Time(src.stamp_ns);
  tf2::convert(src.pose, dest.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      dest.covariance[r * 6 + c] = src.covariance(r, c);
    }
  }
}

void fromMsg(const pose_graph_msgs::PoseGraphEdge& src, PoseGraphEdge& dest) {
  dest.type = static_cast<PoseGraphEdge::Type>(src.type);
  dest.key_from = src.key_from;
  dest.key_to = src.key_to;
  dest.robot_from = src.robot_from;
  dest.robot_to = src.robot_to;
  dest.stamp_ns = rclcpp::Time(src.header.stamp).nanoseconds();
  tf2::convert(src.pose, dest.pose);

  // Store covariance in row-major order.
  for (size_t r = 0; r < 6; ++r) {
    for (size_t c = 0; c < 6; ++c) {
      dest.covariance(r, c) = src.covariance[r * 6 + c];
    }
  }
}

void toMsg(const PoseGraphNode& src, pose_graph_msgs::PoseGraphNode& dest) {
  dest.header.stamp = rclcpp::Time(src.stamp_ns);
  dest.header.frame_id = src.frame_id;
  dest.key = src.key;
  dest.robot_id = src.robot_id;
  tf2::convert(src.pose, dest.pose);
}

void fromMsg(const pose_graph_msgs::PoseGraphNode& src, PoseGraphNode& dest) {
  dest.stamp_ns = rclcpp::Time(src.header.stamp).nanoseconds();
  dest.frame_id = src.header.frame_id;
  dest.key = src.key;
  dest.robot_id = src.robot_id;
  tf2::convert(src.pose, dest.pose);
}

void toMsg(const PoseGraph& src, pose_graph_msgs::PoseGraph& dest) {
  dest.header.stamp = rclcpp::Time(src.stamp_ns);
  dest.header.frame_id = src.frame_id;

  dest.nodes.reserve(src.nodes.size());
  for (const auto& node : src.nodes) {
    auto& dest_node = dest.nodes.emplace_back();
    toMsg(node, dest_node);
  }

  dest.edges.reserve(src.edges.size());
  for (const auto& edge : src.edges) {
    auto& dest_edge = dest.edges.emplace_back();
    toMsg(edge, dest_edge);
  }
}

void fromMsg(const pose_graph_msgs::PoseGraph& src, PoseGraph& dest) {
  dest.stamp_ns = rclcpp::Time(src.header.stamp).nanoseconds();
  dest.frame_id = src.header.frame_id;

  dest.nodes.reserve(src.nodes.size());
  for (const auto& node : src.nodes) {
    auto& dest_node = dest.nodes.emplace_back();
    fromMsg(node, dest_node);
  }

  dest.edges.reserve(src.edges.size());
  for (const auto& edge : src.edges) {
    auto& dest_edge = dest.edges.emplace_back();
    fromMsg(edge, dest_edge);
  }
}

}  // namespace pose_graph_tools

namespace rclcpp {

using PoseGraphAdapter = TypeAdapter<pose_graph_tools::PoseGraph,
                                     pose_graph_tools_msgs::msg::PoseGraph>;

void PoseGraphAdapter::convert_to_ros_message(const custom_type& src,
                                              ros_message_type& dest) {
  pose_graph_tools::toMsg(src, dest);
}

void PoseGraphAdapter::convert_to_custom(const ros_message_type& src,
                                         custom_type& dest) {
  pose_graph_tools::fromMsg(src, dest);
}

}  // namespace rclcpp
