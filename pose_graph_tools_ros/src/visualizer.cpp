#include "pose_graph_tools_ros/visualizer.h"

#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "geometry_msgs/Quaternion.h"
#include "interactive_markers/interactive_marker_server.h"

Visualizer::Visualizer(const ros::NodeHandle& nh) {
  ROS_INFO("Initializing pose graph visualizer");

  // start subscribers
  ros::NodeHandle nl(nh);
  pose_graph_sub_ = nl.subscribe<pose_graph_tools_msgs::PoseGraph>(
      "graph", 10, &Visualizer::PoseGraphCallback, this);

  marker_array_pub_ = nl.advertise<visualization_msgs::MarkerArray>(
      "pose_graph_markers", 10, false);

  interactive_mrkr_srvr_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "interactive_node", "", false);

  ros::spin();
}

void Visualizer::PoseGraphCallback(
    const pose_graph_tools_msgs::PoseGraph::ConstPtr& msg) {
  // iterate through nodes in pose graph
  for (const auto& msg_node : msg->nodes) {
    // Fill pose nodes (representing the robot position)
    keyed_poses_[msg_node.robot_id][msg_node.key] = msg_node.pose;
  }

  // update frame id
  frame_id_ = msg->header.frame_id;

  odometry_edges_.clear();
  loop_edges_.clear();
  rejected_loop_edges_.clear();
  // iterate through edges in pose graph
  for (const auto& msg_edge : msg->edges) {
    Node from = std::make_pair(msg_edge.robot_from, msg_edge.key_from);
    Node to = std::make_pair(msg_edge.robot_to, msg_edge.key_to);
    if (msg_edge.type == pose_graph_tools_msgs::PoseGraphEdge::ODOM) {
      // initialize first seen robot id
      odometry_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type ==
               pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type ==
               pose_graph_tools_msgs::PoseGraphEdge::REJECTED_LOOPCLOSE) {
      rejected_loop_edges_.emplace_back(std::make_pair(from, to));
    }
  }

  visualize();
}

geometry_msgs::Point positionFromKey(
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses,
    int robot_id,
    uint64_t key) {
  return keyed_poses.at(robot_id).at(key).position;
}

geometry_msgs::Quaternion orientationFromKey(
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses,
    int robot_id,
    uint64_t key) {
  return keyed_poses.at(robot_id).at(key).orientation;
}

geometry_msgs::Point Visualizer::getPositionFromKey(int robot_id,
                                                    uint64_t key) const {
  return keyed_poses_.at(robot_id).at(key).position;
}

// Interactive Marker Menu to click and see key of node
void MakeMenuMarker(
    const std::string frame_id,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer>
        interactive_mrkr_srvr,
    const geometry_msgs::Pose& position,
    const std::string& id_number) {
  interactive_markers::MenuHandler menu_handler;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1.0;
  int_marker.pose = position;
  int_marker.name = id_number;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1.0;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  interactive_mrkr_srvr->insert(int_marker);
  menu_handler.apply(*interactive_mrkr_srvr, int_marker.name);
}

visualization_msgs::Marker build_odom_marker(
    std::string frame_id,
    std::vector<Visualizer::Edge> odom_edges,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.ns = "odom_edges";
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  for (size_t ii = 0; ii < odom_edges.size(); ++ii) {
    int robot_id = odom_edges[ii].first.first;
    const auto key1 = odom_edges[ii].first.second;
    const auto key2 = odom_edges[ii].second.second;

    // TODO(Yun) currently the below color formula
    // means that only support up to 5 robots
    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(robot_id) / 5;
    color.g = 1 - static_cast<float>(robot_id) / 5;
    color.b = 0.0;
    color.a = 0.8;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;

    m.points.push_back(positionFromKey(keyed_poses, robot_id, key1));
    m.points.push_back(positionFromKey(keyed_poses, robot_id, key2));
    m.colors.push_back(color);
    m.colors.push_back(color);
  }
  return m;
}

std::vector<visualization_msgs::Marker> build_heading_markers(
    std::string frame_id,
    std::vector<Visualizer::Edge> odom_edges,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  std::vector<visualization_msgs::Marker> arrows;

  int id = 0;
  for (size_t ii = 0; ii < odom_edges.size(); ++ii) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.ns = "heading";
    m.id = id++;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::ARROW;
    int robot_id = odom_edges[ii].first.first;
    const auto key = odom_edges[ii].first.second;

    // TODO(Yun) currently the below color formula
    // means that only support up to 5 robots
    std_msgs::ColorRGBA color;
    m.color.r = 1 - static_cast<float>(robot_id) / 5;
    m.color.g = static_cast<float>(robot_id) / 5;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.pose.orientation.w = 1.0;

    auto pos = positionFromKey(keyed_poses, robot_id, key);
    auto quat = orientationFromKey(keyed_poses, robot_id, key);
    m.pose.position = pos;
    m.pose.orientation = quat;
    arrows.push_back(m);
  }
  return arrows;
}

visualization_msgs::Marker build_lc_marker(
    std::string frame_id,
    std::vector<Visualizer::Edge> loop_edges,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  // Publish loop closure edges.
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.ns = "LoopClosures";
  m.id = 1;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.color.r = 0.0;
  m.color.g = 0.2;
  m.color.b = 1.0;
  m.color.a = 0.8;
  m.scale.x = 0.02;
  m.pose.orientation.w = 1.0;

  for (size_t ii = 0; ii < loop_edges.size(); ++ii) {
    const auto robot1 = loop_edges[ii].first.first;
    const auto robot2 = loop_edges[ii].second.first;
    const auto key1 = loop_edges[ii].first.second;
    const auto key2 = loop_edges[ii].second.second;

    m.points.push_back(positionFromKey(keyed_poses, robot1, key1));
    m.points.push_back(positionFromKey(keyed_poses, robot2, key2));
  }
  return m;
}

visualization_msgs::Marker build_rejected_lc_marker(
    std::string frame_id,
    std::vector<Visualizer::Edge> loop_edges,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  // Publish loop closure edges.
  // Publish the rejected loop closure edges
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.ns = "RejectedLoopClosures";
  m.id = 1;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.color.r = 0.5;
  m.color.g = 0.5;
  m.color.b = 0.5;
  m.color.a = 0.7;
  m.scale.x = 0.02;
  m.pose.orientation.w = 1.0;

  for (size_t ii = 0; ii < loop_edges.size(); ++ii) {
    const auto robot1 = loop_edges[ii].first.first;
    const auto robot2 = loop_edges[ii].second.first;
    const auto key1 = loop_edges[ii].first.second;
    const auto key2 = loop_edges[ii].second.second;

    m.points.push_back(positionFromKey(keyed_poses, robot1, key1));
    m.points.push_back(positionFromKey(keyed_poses, robot2, key2));
  }
  return m;
}

std::vector<visualization_msgs::Marker> build_node_ids(
    std::string frame_id,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  // Publish loop closure edges.
  // Publish node IDs in the pose graph.

  std::vector<visualization_msgs::Marker> m_ids;
  int id_base = 100;
  for (const auto& robot : keyed_poses) {
    for (const auto& keyedPose : robot.second) {
      visualization_msgs::Marker m;
      m.header.frame_id = frame_id;
      m.ns = "NodeIds";
      m.action = visualization_msgs::Marker::ADD;
      m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.2;
      m.color.a = 0.8;
      m.scale.z =
          0.01;  // Only Scale z is used - height of capital A in the text
      m.pose.orientation.w = 1.0;

      m.pose = keyedPose.second;
      // Display text for the node
      std::string robot_id = std::to_string(keyedPose.first);
      m.text = robot_id;
      m.id = id_base + keyedPose.first;
      m_ids.push_back(m);
    }
  }
  return m_ids;
}

visualization_msgs::Marker build_keyframes(
    std::string frame_id,
    std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses) {
  // Publish keyframe nodes in the pose graph.
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.ns = "keyframes";
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.3;
  m.color.a = 0.8;
  m.scale.x = 0.05;
  m.scale.y = 0.05;
  m.scale.z = 0.05;
  m.pose.orientation.w = 1.0;

  for (const auto& robot : keyed_poses) {
    for (const auto& keyedPose : robot.second) {
      m.points.push_back(
          positionFromKey(keyed_poses, robot.first, keyedPose.first));
    }
  }
  return m;
}

void Visualizer::visualize() {
  visualization_msgs::MarkerArray ma;
  if (marker_array_pub_.getNumSubscribers() <= 0) {
    return;
  }

  // Odometry Edges
  visualization_msgs::Marker m_odom =
      build_odom_marker(frame_id_, odometry_edges_, keyed_poses_);
  ma.markers.push_back(m_odom);

  // Heading (orientation) Edges
  std::vector<visualization_msgs::Marker> m_headings =
      build_heading_markers(frame_id_, odometry_edges_, keyed_poses_);
  ma.markers.insert(ma.markers.end(), m_headings.begin(), m_headings.end());

  // Loop Closures
  visualization_msgs::Marker m_lc =
      build_lc_marker(frame_id_, loop_edges_, keyed_poses_);
  ma.markers.push_back(m_lc);

  // Rejected Loop Closures
  visualization_msgs::Marker m_rejected_lc =
      build_rejected_lc_marker(frame_id_, rejected_loop_edges_, keyed_poses_);
  ma.markers.push_back(m_rejected_lc);

  // Node IDs
  std::vector<visualization_msgs::Marker> m_node_ids =
      build_node_ids(frame_id_, keyed_poses_);
  ma.markers.insert(ma.markers.end(), m_node_ids.begin(), m_node_ids.end());

  // Keyframes
  visualization_msgs::Marker m_keyframes =
      build_keyframes(frame_id_, keyed_poses_);
  ma.markers.push_back(m_keyframes);

  // Publish it all!
  marker_array_pub_.publish(ma);

  // Interactive Markers
  for (const auto& robot : keyed_poses_) {
    for (const auto& keyedPose : robot.second) {
      // Display text for the node
      std::string robot_id = std::to_string(keyedPose.first);
      MakeMenuMarker(
          frame_id_, interactive_mrkr_srvr_, keyedPose.second, robot_id);
    }
  }

  if (interactive_mrkr_srvr_ != nullptr) {
    interactive_mrkr_srvr_->applyChanges();
  }
}
