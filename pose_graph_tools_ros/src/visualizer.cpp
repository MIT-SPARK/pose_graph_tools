#include "pose_graph_tools_ros/visualizer.h"

#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pose_graph_tools_ros {

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using pose_graph_tools_msgs::msg::PoseGraph;
using pose_graph_tools_msgs::msg::PoseGraphEdge;
using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using KeyedPoses = std::map<int, std::map<uint64_t, geometry_msgs::msg::Pose>>;
using VisualizerEdges = std::vector<Visualizer::Edge>;

namespace {

std_msgs::msg::ColorRGBA makeColor(int robot_id) {
  // TODO(Yun) currently the below color formula
  // means that only support up to 5 robots
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(robot_id) / 5;
  color.g = 1 - static_cast<float>(robot_id) / 5;
  color.b = 0.0;
  color.a = 0.8;
  return color;
}

Point positionFromKey(const KeyedPoses& keyed_poses,
                      int robot_id,
                      uint64_t key) {
  return keyed_poses.at(robot_id).at(key).position;
}

Quaternion orientationFromKey(const KeyedPoses& keyed_poses,
                              int robot_id,
                              uint64_t key) {
  return keyed_poses.at(robot_id).at(key).orientation;
}

void makeMenuMarker(interactive_markers::InteractiveMarkerServer& server,
                    const std::string& frame_id,
                    const Pose& position,
                    const std::string& id_number) {
  interactive_markers::MenuHandler menu_handler;

  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1.0;
  int_marker.pose = position;
  int_marker.name = id_number;

  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1.0;

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  server.insert(int_marker);
  menu_handler.apply(server, int_marker.name);
}

Marker build_odom_marker(const KeyedPoses& keyed_poses,
                         const VisualizerEdges& odom_edges,
                         const std::string& frame_id) {
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "odom_edges";
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.type = Marker::LINE_LIST;
  for (size_t ii = 0; ii < odom_edges.size(); ++ii) {
    int robot_id = odom_edges[ii].first.first;
    const auto key1 = odom_edges[ii].first.second;
    const auto key2 = odom_edges[ii].second.second;

    marker.scale.x = 0.02;
    marker.pose.orientation.w = 1.0;

    marker.points.push_back(positionFromKey(keyed_poses, robot_id, key1));
    marker.points.push_back(positionFromKey(keyed_poses, robot_id, key2));
    marker.colors.push_back(makeColor(robot_id));
    marker.colors.push_back(makeColor(robot_id));
  }

  return marker;
}

std::vector<Marker> build_heading_markers(const KeyedPoses& keyed_poses,
                                          const VisualizerEdges& odom_edges,
                                          const std::string& frame_id) {
  std::vector<Marker> arrows;

  int id = 0;
  for (size_t ii = 0; ii < odom_edges.size(); ++ii) {
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "heading";
    marker.id = id++;
    marker.action = Marker::ADD;
    marker.type = Marker::ARROW;
    int robot_id = odom_edges[ii].first.first;
    const auto key = odom_edges[ii].first.second;

    marker.color = makeColor(robot_id);
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.orientation.w = 1.0;

    auto pos = positionFromKey(keyed_poses, robot_id, key);
    auto quat = orientationFromKey(keyed_poses, robot_id, key);
    marker.pose.position = pos;
    marker.pose.orientation = quat;
    arrows.push_back(marker);
  }

  return arrows;
}

Marker build_lc_marker(const KeyedPoses& keyed_poses,
                       const VisualizerEdges& loop_edges,
                       const std::string& frame_id) {
  // Publish loop closure edges.
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "LoopClosures";
  marker.id = 1;
  marker.action = Marker::ADD;
  marker.type = Marker::LINE_LIST;
  marker.color.r = 0.0;
  marker.color.g = 0.2;
  marker.color.b = 1.0;
  marker.color.a = 0.8;
  marker.scale.x = 0.02;
  marker.pose.orientation.w = 1.0;

  for (size_t ii = 0; ii < loop_edges.size(); ++ii) {
    const auto robot1 = loop_edges[ii].first.first;
    const auto robot2 = loop_edges[ii].second.first;
    const auto key1 = loop_edges[ii].first.second;
    const auto key2 = loop_edges[ii].second.second;

    marker.points.push_back(positionFromKey(keyed_poses, robot1, key1));
    marker.points.push_back(positionFromKey(keyed_poses, robot2, key2));
  }

  return marker;
}

Marker build_rejected_lc_marker(const KeyedPoses& keyed_poses,
                                const VisualizerEdges& loop_edges,
                                const std::string& frame_id) {
  // Publish loop closure edges.
  // Publish the rejected loop closure edges
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "RejectedLoopClosures";
  marker.id = 1;
  marker.action = Marker::ADD;
  marker.type = Marker::LINE_LIST;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;
  marker.scale.x = 0.02;
  marker.pose.orientation.w = 1.0;

  for (size_t ii = 0; ii < loop_edges.size(); ++ii) {
    const auto robot1 = loop_edges[ii].first.first;
    const auto robot2 = loop_edges[ii].second.first;
    const auto key1 = loop_edges[ii].first.second;
    const auto key2 = loop_edges[ii].second.second;

    marker.points.push_back(positionFromKey(keyed_poses, robot1, key1));
    marker.points.push_back(positionFromKey(keyed_poses, robot2, key2));
  }

  return marker;
}

std::vector<Marker> build_node_ids(const KeyedPoses& keyed_poses,
                                   const std::string& frame_id) {
  // Publish loop closure edges.
  // Publish node IDs in the pose graph.

  std::vector<Marker> m_ids;
  int id_base = 100;
  for (const auto& robot : keyed_poses) {
    for (const auto& [key, pose] : robot.second) {
      auto& marker = m_ids.emplace_back();
      marker.header.frame_id = frame_id;
      marker.ns = "NodeIds";
      marker.action = Marker::ADD;
      marker.type = Marker::TEXT_VIEW_FACING;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.2;
      marker.color.a = 0.8;
      // Only Scale z is used - height of capital A in the text
      marker.scale.z = 0.01;
      marker.pose.orientation.w = 1.0;
      marker.pose = pose;
      // Display text for the node
      std::string robot_id = std::to_string(key);
      marker.text = robot_id;
      marker.id = id_base + key;
    }
  }

  return m_ids;
}

Marker build_keyframes(const KeyedPoses& keyed_poses,
                       const std::string& frame_id) {
  // Publish keyframe nodes in the pose graph.
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "keyframes";
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.type = Marker::SPHERE_LIST;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.3;
  marker.color.a = 0.8;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.pose.orientation.w = 1.0;

  for (const auto& robot : keyed_poses) {
    for (const auto& keyedPose : robot.second) {
      marker.points.push_back(
          positionFromKey(keyed_poses, robot.first, keyedPose.first));
    }
  }
  return marker;
}

}  // namespace

Visualizer::Visualizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pose_graph_visualizer", options) {
  RCLCPP_INFO(get_logger(), "Initializing pose graph visualizer");

  // start subscribers
  sub_ = create_subscription<PoseGraph>(
      "graph", 10, [this](const PoseGraph& msg) -> void { callback(msg); });
  pub_ = create_publisher<MarkerArray>("pose_graph_markers", 10);
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "interactive_node", this);
}

void Visualizer::callback(const PoseGraph& msg) {
  // iterate through nodes in pose graph
  for (const auto& msg_node : msg.nodes) {
    // Fill pose nodes (representing the robot position)
    keyed_poses_[msg_node.robot_id][msg_node.key] = msg_node.pose;
  }

  // update frame id
  frame_id_ = msg.header.frame_id;

  odometry_edges_.clear();
  loop_edges_.clear();
  rejected_loop_edges_.clear();
  // iterate through edges in pose graph
  for (const auto& msg_edge : msg.edges) {
    Node from = std::make_pair(msg_edge.robot_from, msg_edge.key_from);
    Node to = std::make_pair(msg_edge.robot_to, msg_edge.key_to);
    if (msg_edge.type == PoseGraphEdge::ODOM) {
      // initialize first seen robot id
      odometry_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type == PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type == PoseGraphEdge::REJECTED_LOOPCLOSE) {
      rejected_loop_edges_.emplace_back(std::make_pair(from, to));
    }
  }

  visualize();
}

void Visualizer::visualize() {
  if (!pub_->get_subscription_count()) {
    return;
  }

  MarkerArray ma;
  // Odometry Edges
  auto m_odom = build_odom_marker(keyed_poses_, odometry_edges_, frame_id_);
  ma.markers.push_back(m_odom);

  // Heading (orientation) Edges
  auto m_headings =
      build_heading_markers(keyed_poses_, odometry_edges_, frame_id_);
  ma.markers.insert(ma.markers.end(), m_headings.begin(), m_headings.end());

  // Loop Closures
  auto m_lc = build_lc_marker(keyed_poses_, loop_edges_, frame_id_);
  ma.markers.push_back(m_lc);

  // Rejected Loop Closures
  auto m_rejected_lc =
      build_rejected_lc_marker(keyed_poses_, loop_edges_, frame_id_);
  ma.markers.push_back(m_rejected_lc);

  // Node IDs
  auto m_node_ids = build_node_ids(keyed_poses_, frame_id_);
  ma.markers.insert(ma.markers.end(), m_node_ids.begin(), m_node_ids.end());

  // Keyframes
  auto m_keyframes = build_keyframes(keyed_poses_, frame_id_);
  ma.markers.push_back(m_keyframes);

  // Publish it all!
  pub_->publish(ma);

  if (!server_) {
    return;
  }

  // Interactive Markers
  for (const auto& robot : keyed_poses_) {
    for (const auto& [key, pose] : robot.second) {
      // Display text for the node
      std::string robot_id = std::to_string(key);
      makeMenuMarker(*server_, frame_id_, pose, robot_id);
    }
  }

  server_->applyChanges();
}

}  // namespace pose_graph_tools_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pose_graph_tools_ros::Visualizer)
