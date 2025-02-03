#pragma once

#include <type_traits>

#include <pose_graph_tools/bow_queries.h>
#include <pose_graph_tools/pose_graph.h>
#include <pose_graph_tools_msgs/msg/bow_queries.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/type_adapter.hpp>

namespace pose_graph_tools {

namespace pose_graph_msgs = pose_graph_tools_msgs::msg;

// Conversions for the pose_graph_tools C++ types to and from ROS messages.

void toMsg(const BowVector& src, pose_graph_msgs::BowVector& dest);
void fromMsg(const pose_graph_msgs::BowVector& src, BowVector& dest);

void toMsg(const BowQuery& src, pose_graph_msgs::BowQuery& dest);
void fromMsg(const pose_graph_msgs::BowQuery& src, BowQuery& dest);

void toMsg(const BowQueries& src, pose_graph_msgs::BowQueries& dest);
void fromMsg(const pose_graph_msgs::BowQueries& src, BowQueries& dest);

void toMsg(const PoseGraphEdge& src, pose_graph_msgs::PoseGraphEdge& dest);
void fromMsg(const pose_graph_msgs::PoseGraphEdge& src, PoseGraphEdge& dest);

void toMsg(const PoseGraphNode& src, pose_graph_msgs::PoseGraphNode& dest);
void fromMsg(const pose_graph_msgs::PoseGraphNode& src, PoseGraphNode& dest);

void toMsg(const PoseGraph& src, pose_graph_msgs::PoseGraph& dest);
void fromMsg(const pose_graph_msgs::PoseGraph& src, PoseGraph& dest);

}  // namespace pose_graph_tools

namespace rclcpp {

template <>
struct TypeAdapter<pose_graph_tools::PoseGraph,
                   pose_graph_tools_msgs::msg::PoseGraph> {
  using is_specialized = std::true_type;
  using custom_type = pose_graph_tools::PoseGraph;
  using ros_message_type = pose_graph_tools_msgs::msg::PoseGraph;

  static void convert_to_ros_message(const custom_type& src,
                                     ros_message_type& dest);
  static void convert_to_custom(const ros_message_type& src, custom_type& dest);
};

}  // namespace rclcpp

namespace pose_graph_tools {

using PoseGraphTypeAdapter =
    rclcpp::adapt_type<PoseGraph>::as<pose_graph_msgs::PoseGraph>;
using PoseGraphPublisher = rclcpp::Publisher<PoseGraphTypeAdapter>::SharedPtr;
using PoseGraphSubscription =
    rclcpp::Subscription<PoseGraphTypeAdapter>::SharedPtr;

}  // namespace pose_graph_tools
