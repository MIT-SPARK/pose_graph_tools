#pragma once

#include <pose_graph_tools/all.h>
#include <pose_graph_tools_msgs/BowQueries.h>
#include <pose_graph_tools_msgs/PoseGraph.h>

namespace pose_graph_tools {

// Conversions for the pose_graph_tools C++ types to and from ROS messages.

pose_graph_tools_msgs::BowVector toMsg(const BowVector& bow_vector);
BowVector fromMsg(const pose_graph_tools_msgs::BowVector& bow_vector);

pose_graph_tools_msgs::BowQuery toMsg(const BowQuery& bow_query);
BowQuery fromMsg(const pose_graph_tools_msgs::BowQuery& bow_query);

pose_graph_tools_msgs::BowQueries toMsg(const BowQueries& bow_queries);
BowQueries fromMsg(const pose_graph_tools_msgs::BowQueries& bow_queries);

pose_graph_tools_msgs::PoseGraphEdge toMsg(const PoseGraphEdge& pose_graph_edge);
PoseGraphEdge fromMsg(const pose_graph_tools_msgs::PoseGraphEdge& pose_graph_edge);

pose_graph_tools_msgs::PoseGraphNode toMsg(const PoseGraphNode& pose_graph_node);
PoseGraphNode fromMsg(const pose_graph_tools_msgs::PoseGraphNode& pose_graph_node);

pose_graph_tools_msgs::PoseGraph toMsg(const PoseGraph& pose_graph);
PoseGraph fromMsg(const pose_graph_tools_msgs::PoseGraph& pose_graph);

}  // namespace pose_graph_tools
